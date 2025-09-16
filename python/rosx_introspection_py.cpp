#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <rosx_introspection/ros_parser.hpp>
#include <rosx_introspection/contrib/msgpack.hpp>
#include <vector>
#include <memory>
#include <stdexcept>

namespace details
{
int convertToMsgpack(const RosMsgParser::FlatMessage& flat_msg, uint8_t* buffer)
{
  const uint32_t num_elements = flat_msg.value.size() + flat_msg.name.size();
  uint8_t* data_ptr = buffer;

  data_ptr += msgpack::pack_map(data_ptr, num_elements);

  std::string key_str;

  // Write numerical values as key-value pairs
  for (const auto& [key, num_value] : flat_msg.value)
  {
    key.toStr(key_str);
    data_ptr += msgpack::pack_string(data_ptr, key_str);

    switch (num_value.getTypeID())
    {
      case RosMsgParser::BuiltinType::UINT64:
        data_ptr += msgpack::pack_uint(data_ptr, num_value.extract<uint64_t>());
        break;
      case RosMsgParser::BuiltinType::FLOAT64:
        data_ptr += msgpack::pack_double(data_ptr, num_value.extract<double>());
        break;
      case RosMsgParser::BuiltinType::FLOAT32:
        data_ptr += msgpack::pack_float(data_ptr, num_value.extract<float>());
        break;
      case RosMsgParser::BuiltinType::BOOL:
        data_ptr += msgpack::pack_bool(data_ptr, num_value.extract<bool>());
        break;
      default:
        // fallback to int64 for all the other types
        data_ptr += msgpack::pack_int(data_ptr, num_value.convert<int64_t>());
        break;
    }
  }

  // Write string values as key-value pairs
  for (const auto& [key, string_value] : flat_msg.name)
  {
    key.toStr(key_str);
    data_ptr += msgpack::pack_string(data_ptr, key_str);
    data_ptr += msgpack::pack_string(data_ptr, string_value);
  }

  return static_cast<int>(data_ptr - buffer);
}
}  // namespace details

namespace nb = nanobind;

class Parser
{
private:
  RosMsgParser::Parser parser_;
  RosMsgParser::FlatMessage flat_msg_;
  RosMsgParser::NanoCDR_Deserializer deserializer_;
  std::vector<uint8_t> output_buffer_;

public:
  Parser(const std::string& topic_name, const std::string& type_name,
         const std::string& schema)
    : parser_(topic_name, type_name, schema)
    , output_buffer_(1024 * 1024)  // 1MB initial buffer
  {
  }

  ~Parser() = default;

  // Disable copy
  Parser(const Parser&) = delete;
  Parser& operator=(const Parser&) = delete;

  // Enable move
  Parser(Parser&& other) noexcept : parser_(std::move(other.parser_))
  {
  }

  nb::bytes parse_to_msgpack(nb::bytes raw_data)
  {
    // Create span from input data
    RosMsgParser::Span<const uint8_t> msg_span(
        reinterpret_cast<const uint8_t*>(raw_data.c_str()), raw_data.size());

    if (!parser_.deserialize(msg_span, &flat_msg_, &deserializer_))
    {
      throw std::runtime_error("Failed to parse ROS message");
    }

    const int msgpack_size = details::convertToMsgpack(flat_msg_, output_buffer_.data());
    return nb::bytes(reinterpret_cast<const char*>(output_buffer_.data()), msgpack_size);
  }
};

NB_MODULE(rosx_introspection, m)
{
  m.doc() = "Python bindings for rosx_introspection - ROS message parser";

  nb::class_<Parser>(m, "Parser")
      .def(nb::init<const std::string&, const std::string&, const std::string&>(),
           nb::arg("topic_name"), nb::arg("type_name"), nb::arg("schema"),
           "Create a ROS message parser.\n\n"
           "Args:\n"
           "    topic_name: Optional topic name (used as prefix in output)\n"
           "    type_name: ROS message type name\n"
           "    schema: ROS message schema definition")

      .def("parse_to_msgpack", &Parser::parse_to_msgpack, nb::arg("raw_data"),
           "Parse raw ROS message to msgpack format.\n\n"
           "Args:\n"
           "    raw_data: Raw binary ROS message data\n\n"
           "Returns:\n"
           "    bytes: Msgpack-encoded parsed message");
}