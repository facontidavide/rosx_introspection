#include <gtest/gtest.h>

#include "rosx_introspection/deserializer.hpp"
#include "rosx_introspection/msgpack_utils.hpp"
#include "rosx_introspection/ros_message.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/serializer.hpp"

using namespace RosMsgParser;

namespace {

bool HasJsonSupport() {
  Parser parser("topic", ROSType("my_pkg/Test"), "uint32 value\n");
  NanoCDR_Serializer serializer;
  serializer.reset();
  serializer.serialize(UINT32, Variant(uint32_t(0)));

  NanoCDR_Deserializer deserializer;
  std::string json;
  try {
    return parser.deserializeIntoJson(
        Span<const uint8_t>(
            reinterpret_cast<const uint8_t*>(serializer.getBufferData()), serializer.getBufferSize()),
        &json, &deserializer);
  } catch (const std::runtime_error& ex) {
    if (std::string(ex.what()).find("without JSON support") != std::string::npos) {
      return false;
    }
    throw;
  }
}

}  // namespace

TEST(NanoSerializer, RoundTrip) {
  // Create and initialize serializer
  NanoCDR_Serializer serializer;
  serializer.reset();

  // Serialize test data
  serializer.serialize(BOOL, Variant(uint8_t(1)));  // bool is stored as uint8_t in ROS
  serializer.serialize(INT8, Variant(int8_t(-42)));
  serializer.serialize(UINT8, Variant(uint8_t(200)));
  serializer.serialize(INT16, Variant(int16_t(-1000)));
  serializer.serialize(UINT16, Variant(uint16_t(50000)));
  serializer.serialize(INT32, Variant(int32_t(-100000)));
  serializer.serialize(UINT32, Variant(uint32_t(3000000000)));
  serializer.serialize(INT64, Variant(int64_t(-9223372036854775807LL)));
  serializer.serialize(UINT64, Variant(uint64_t(18446744073709551615ULL)));
  serializer.serialize(FLOAT32, Variant(float(3.14159f)));
  serializer.serialize(FLOAT64, Variant(double(2.718281828)));
  serializer.serializeString("Hello, ROS!");

  // Time (seconds and nanoseconds)
  serializer.serialize(UINT32, Variant(uint32_t(1234567890)));  // seconds
  serializer.serialize(UINT32, Variant(uint32_t(123456789)));   // nanoseconds

  // Duration (seconds and nanoseconds)
  serializer.serialize(INT32, Variant(int32_t(3600)));       // seconds
  serializer.serialize(INT32, Variant(int32_t(500000000)));  // nanoseconds

  // int32 array with 3 elements
  serializer.serializeUInt32(3);  // array size
  serializer.serialize(INT32, Variant(int32_t(10)));
  serializer.serialize(INT32, Variant(int32_t(20)));
  serializer.serialize(INT32, Variant(int32_t(30)));

  // float64[3] fixed array
  serializer.serialize(FLOAT64, Variant(double(1.1)));
  serializer.serialize(FLOAT64, Variant(double(2.2)));
  serializer.serialize(FLOAT64, Variant(double(3.3)));

  // string array with 2 elements
  serializer.serializeUInt32(2);  // array size
  serializer.serializeString("first");
  serializer.serializeString("second");

  // byte array
  serializer.serializeUInt32(5);  // array size
  serializer.serialize(UINT8, Variant(uint8_t(0xAA)));
  serializer.serialize(UINT8, Variant(uint8_t(0xBB)));
  serializer.serialize(UINT8, Variant(uint8_t(0xCC)));
  serializer.serialize(UINT8, Variant(uint8_t(0xDD)));
  serializer.serialize(UINT8, Variant(uint8_t(0xEE)));

  // Get the serialized buffer
  const char* buffer_data = serializer.getBufferData();
  size_t buffer_size = serializer.getBufferSize();

  // Create a span from the buffer
  std::vector<uint8_t> buffer_copy(buffer_data, buffer_data + buffer_size);
  Span<const uint8_t> buffer_span(buffer_copy.data(), buffer_copy.size());

  // Deserialize the buffer
  NanoCDR_Deserializer deserializer;
  deserializer.init(buffer_span);

  // Verify deserialized values match original
  EXPECT_EQ(deserializer.deserialize(BOOL).convert<uint8_t>(),
            1);  // bool is uint8_t in ROS
  EXPECT_EQ(deserializer.deserialize(INT8).convert<int8_t>(), -42);
  EXPECT_EQ(deserializer.deserialize(UINT8).convert<uint8_t>(), 200);
  EXPECT_EQ(deserializer.deserialize(INT16).convert<int16_t>(), -1000);
  EXPECT_EQ(deserializer.deserialize(UINT16).convert<uint16_t>(), 50000);
  EXPECT_EQ(deserializer.deserialize(INT32).convert<int32_t>(), -100000);
  EXPECT_EQ(deserializer.deserialize(UINT32).convert<uint32_t>(), 3000000000);
  EXPECT_EQ(deserializer.deserialize(INT64).convert<int64_t>(), -9223372036854775807LL);
  EXPECT_EQ(deserializer.deserialize(UINT64).convert<uint64_t>(), 18446744073709551615ULL);
  EXPECT_FLOAT_EQ(deserializer.deserialize(FLOAT32).convert<float>(), 3.14159f);
  EXPECT_DOUBLE_EQ(deserializer.deserialize(FLOAT64).convert<double>(), 2.718281828);

  std::string str_value;
  deserializer.deserializeString(str_value);
  EXPECT_EQ(str_value, "Hello, ROS!");

  // Time
  EXPECT_EQ(deserializer.deserialize(UINT32).convert<uint32_t>(), 1234567890);
  EXPECT_EQ(deserializer.deserialize(UINT32).convert<uint32_t>(), 123456789);

  // Duration
  EXPECT_EQ(deserializer.deserialize(INT32).convert<int32_t>(), 3600);
  EXPECT_EQ(deserializer.deserialize(INT32).convert<int32_t>(), 500000000);

  // int32 array
  EXPECT_EQ(deserializer.deserializeUInt32(), 3);
  EXPECT_EQ(deserializer.deserialize(INT32).convert<int32_t>(), 10);
  EXPECT_EQ(deserializer.deserialize(INT32).convert<int32_t>(), 20);
  EXPECT_EQ(deserializer.deserialize(INT32).convert<int32_t>(), 30);

  // float64[3] fixed array
  EXPECT_DOUBLE_EQ(deserializer.deserialize(FLOAT64).convert<double>(), 1.1);
  EXPECT_DOUBLE_EQ(deserializer.deserialize(FLOAT64).convert<double>(), 2.2);
  EXPECT_DOUBLE_EQ(deserializer.deserialize(FLOAT64).convert<double>(), 3.3);

  // string array
  EXPECT_EQ(deserializer.deserializeUInt32(), 2);
  std::string str1, str2;
  deserializer.deserializeString(str1);
  deserializer.deserializeString(str2);
  EXPECT_EQ(str1, "first");
  EXPECT_EQ(str2, "second");

  // byte array
  EXPECT_EQ(deserializer.deserializeUInt32(), 5);
  EXPECT_EQ(deserializer.deserialize(UINT8).convert<uint8_t>(), 0xAA);
  EXPECT_EQ(deserializer.deserialize(UINT8).convert<uint8_t>(), 0xBB);
  EXPECT_EQ(deserializer.deserialize(UINT8).convert<uint8_t>(), 0xCC);
  EXPECT_EQ(deserializer.deserialize(UINT8).convert<uint8_t>(), 0xDD);
  EXPECT_EQ(deserializer.deserialize(UINT8).convert<uint8_t>(), 0xEE);

  // Verify all bytes have been consumed
  EXPECT_EQ(deserializer.bytesLeft(), 0);
}

TEST(NanoDeserializer, EmptyByteSequence) {
  NanoCDR_Serializer serializer;
  serializer.reset();
  serializer.serializeUInt32(0);

  NanoCDR_Deserializer deserializer;
  deserializer.init(
      Span<const uint8_t>(reinterpret_cast<const uint8_t*>(serializer.getBufferData()), serializer.getBufferSize()));

  Span<const uint8_t> bytes;
  EXPECT_NO_THROW(bytes = deserializer.deserializeByteSequence());
  EXPECT_EQ(bytes.size(), 0u);
  EXPECT_EQ(deserializer.bytesLeft(), 0u);
}

TEST(ParserJson, LargeArrayShouldNotCorruptFollowingFields) {
  if (!HasJsonSupport()) {
    GTEST_SKIP() << "JSON support disabled in this build";
  }

  Parser parser("topic", ROSType("my_pkg/Test"), "uint8[] data\nuint32 tail\n");

  NanoCDR_Serializer serializer;
  serializer.reset();
  serializer.serializeUInt32(101);
  for (int i = 0; i < 101; i++) {
    serializer.serialize(UINT8, Variant(uint8_t(i)));
  }
  serializer.serialize(UINT32, Variant(uint32_t(42)));

  NanoCDR_Deserializer deserializer;
  std::string json;
  ASSERT_TRUE(parser.deserializeIntoJson(
      Span<const uint8_t>(reinterpret_cast<const uint8_t*>(serializer.getBufferData()), serializer.getBufferSize()),
      &json, &deserializer));

  EXPECT_NE(json.find("\"tail\":42"), std::string::npos);
}

TEST(ParserFlatMessage, LargeUint8ArrayShouldBeBlob) {
  Parser parser("topic", ROSType("my_pkg/Test"), "uint8[] data\n");
  parser.setMaxArrayPolicy(Parser::DISCARD_LARGE_ARRAYS, 100);
  parser.setBlobPolicy(Parser::STORE_BLOB_AS_COPY);

  NanoCDR_Serializer serializer;
  serializer.reset();
  serializer.serializeUInt32(101);
  for (int i = 0; i < 101; i++) {
    serializer.serialize(UINT8, Variant(uint8_t(i)));
  }

  FlatMessage flat;
  NanoCDR_Deserializer deserializer;
  ASSERT_TRUE(parser.deserialize(
      Span<const uint8_t>(reinterpret_cast<const uint8_t*>(serializer.getBufferData()), serializer.getBufferSize()),
      &flat, &deserializer));

  EXPECT_EQ(flat.blob.size(), 1u);
  EXPECT_EQ(flat.value.size(), 0u);
}

TEST(ParserJson, NegativeInt8ShouldNotAbort) {
  if (!HasJsonSupport()) {
    GTEST_SKIP() << "JSON support disabled in this build";
  }

  ASSERT_EXIT(
      {
        Parser parser("topic", ROSType("my_pkg/Test"), "int8 value\n");
        NanoCDR_Serializer serializer;
        parser.serializeFromJson(R"({"value":-1})", &serializer);

        NanoCDR_Deserializer deserializer;
        deserializer.init(Span<const uint8_t>(
            reinterpret_cast<const uint8_t*>(serializer.getBufferData()), serializer.getBufferSize()));
        const auto decoded = deserializer.deserialize(INT8).convert<int8_t>();
        if (decoded != -1) {
          std::_Exit(2);
        }
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST(ParserJson, OmittedBoolShouldDefaultToFalse) {
  if (!HasJsonSupport()) {
    GTEST_SKIP() << "JSON support disabled in this build";
  }

  ASSERT_EXIT(
      {
        Parser parser("topic", ROSType("my_pkg/Test"), "bool flag\n");
        NanoCDR_Serializer serializer;
        parser.serializeFromJson("{}", &serializer);

        NanoCDR_Deserializer deserializer;
        deserializer.init(Span<const uint8_t>(
            reinterpret_cast<const uint8_t*>(serializer.getBufferData()), serializer.getBufferSize()));
        const auto decoded = deserializer.deserialize(BOOL).convert<uint8_t>();
        if (decoded != 0) {
          std::_Exit(2);
        }
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST(ParserJson, MalformedJsonShouldNotAbort) {
  if (!HasJsonSupport()) {
    GTEST_SKIP() << "JSON support disabled in this build";
  }

  ASSERT_EXIT(
      {
        Parser parser("topic", ROSType("my_pkg/Test"), "uint32 value\n");
        NanoCDR_Serializer serializer;
        try {
          parser.serializeFromJson("{", &serializer);
        } catch (...) {
          std::_Exit(0);
        }
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST(Msgpack, LargeInputShouldNotCrash) {
  ASSERT_EXIT(
      {
        FlatMessage flat;
        flat.value.reserve(200000);
        for (int i = 0; i < 200000; i++) {
          flat.value.emplace_back(FieldsVector(), Variant(int64_t(i)));
        }

        std::vector<uint8_t> msgpack;
        convertToMsgpack(flat, msgpack);
        if (msgpack.empty()) {
          std::_Exit(2);
        }
        std::_Exit(0);
      },
      ::testing::ExitedWithCode(0), "");
}

TEST(ROSDeserializer, UnsupportedTypeShouldThrow) {
  ROS_Deserializer deserializer;
  std::vector<uint8_t> buffer(1, 0);
  deserializer.init(Span<const uint8_t>(buffer.data(), buffer.size()));

  EXPECT_THROW(deserializer.deserialize(OTHER), std::runtime_error);
}
