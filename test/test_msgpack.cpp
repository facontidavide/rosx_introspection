#include <gtest/gtest.h>

#include <string>
#include <string_view>
#include <vector>

#include "rosx_introspection/deserializer.hpp"
#include "rosx_introspection/msgpack_utils.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/serializer.hpp"

using namespace RosMsgParser;

// Regression for the convertToMsgpack() buffer under-reservation bug: string
// VALUES (unlike numbers) can be arbitrarily large. The loop only reserved 9
// bytes for the value, so pack_string() overflowed msgpack_data and the trailing
// resize(offset) reallocated and zero-filled the overflowed tail, truncating any
// string longer than ~64KB to NULs.
TEST(Msgpack, LargeStringValueSurvivesConversion)
{
  Parser parser("topic", ROSType("test_msgs/BigString"), "string data");

  std::string big(100000, 'x');
  const std::string tail_marker = "ROSX_TAIL_MARKER_END";
  big += tail_marker;  // placed beyond the ~64KB truncation boundary
  ASSERT_GT(big.size(), 64u * 1024u);

  NanoCDR_Serializer enc;
  enc.serializeString(big);
  std::vector<uint8_t> buffer(enc.getBufferData(), enc.getBufferData() + enc.getBufferSize());

  FlatMessage flat;
  NanoCDR_Deserializer deser;
  parser.deserialize(Span<const uint8_t>(buffer.data(), buffer.size()), &flat, &deser);

  // The decoded FlatMessage value itself is correct (the bug was downstream).
  ASSERT_EQ(flat.value.size(), 1u);
  ASSERT_EQ(flat.value[0].second.convert<std::string>(), big);

  std::vector<uint8_t> msgpack_data;
  convertToMsgpack(flat, msgpack_data);

  // The full string -- including its tail past 64KB -- must survive encoding.
  std::string_view sv(reinterpret_cast<const char*>(msgpack_data.data()), msgpack_data.size());
  EXPECT_NE(sv.find(tail_marker), std::string_view::npos)
      << "string value was truncated by convertToMsgpack";
}

// Regression for the oversized-struct-array bug: an array of variable-size
// structs whose length exceeds max_array_size (default 100) must be DISCARDED
// (DISCARD_LARGE_ARRAYS) without throwing, while the read cursor stays aligned so
// that fields AFTER the array are still decoded correctly. deserialize() returns
// false here only to signal "not entirely stored", which must not be treated as a
// hard failure by callers (the Python binding used to throw on it).
TEST(Msgpack, OversizedStructArrayIsDiscardedNotFailed)
{
  const std::string def =
      "Item[] items\n"
      "int32 trailer\n"
      "================================================================================\n"
      "MSG: test_msgs/Item\n"
      "string name\n"
      "float64 value\n";

  Parser parser("topic", ROSType("test_msgs/Outer"), def);

  const int N = 150;  // > max_array_size (100)
  NanoCDR_Serializer enc;
  enc.serializeUInt32(static_cast<uint32_t>(N));
  for (int i = 0; i < N; ++i)
  {
    enc.serializeString("item_" + std::to_string(i));
    enc.serialize(BuiltinType::FLOAT64, double(i));
  }
  enc.serialize(BuiltinType::INT32, int32_t(12345));  // trailer, after the big array

  std::vector<uint8_t> buffer(enc.getBufferData(), enc.getBufferData() + enc.getBufferSize());

  FlatMessage flat;
  NanoCDR_Deserializer deser;
  bool complete = true;
  ASSERT_NO_THROW({
    complete = parser.deserialize(Span<const uint8_t>(buffer.data(), buffer.size()), &flat, &deser);
  });

  // false == "some arrays were discarded" (NOT a parse failure).
  EXPECT_FALSE(complete);

  // Cursor stayed aligned past the 150 discarded structs: the trailer is intact.
  bool found_trailer = false;
  for (const auto& [key, value] : flat.value)
  {
    if (key.toStdString().find("trailer") != std::string::npos)
    {
      EXPECT_EQ(value.convert<int32_t>(), 12345);
      found_trailer = true;
    }
  }
  EXPECT_TRUE(found_trailer) << "trailer field after the oversized array was lost";
}
