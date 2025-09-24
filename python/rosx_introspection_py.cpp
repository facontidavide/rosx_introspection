#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <rosx_introspection/contrib/msgpack.hpp>
#include <rosx_introspection/ros_parser.hpp>
#include <stdexcept>
#include <vector>

namespace details {
void convertToMsgpack(const RosMsgParser::FlatMessage& flat_msg, std::vector<uint8_t>& msgpack_data) {
  msgpack_data.clear();
  msgpack_data.resize(1024 * 64);  // 64KB initial size

  auto resize_if_needed = [&msgpack_data](size_t required_size) {
    if (msgpack_data.size() < required_size) {
      // Resize the buffer if it's not large enough
      msgpack_data.resize(msgpack_data.size() * 2);
    }
  };

  const uint32_t num_elements = flat_msg.value.size();
  uint8_t* data_ptr = msgpack_data.data();

  data_ptr += msgpack::pack_map(data_ptr, num_elements);

  auto pack_variant = [](const RosMsgParser::Variant& number, uint8_t*& data) -> uint32_t {
    switch (number.getTypeID()) {
      case RosMsgParser::BuiltinType::UINT64:
        return msgpack::pack_uint(data, number.extract<uint64_t>());
      case RosMsgParser::BuiltinType::FLOAT64:
        return msgpack::pack_double(data, number.extract<double>());
      case RosMsgParser::BuiltinType::FLOAT32:
        return msgpack::pack_float(data, number.extract<float>());
      case RosMsgParser::BuiltinType::BOOL:
        return msgpack::pack_bool(data, number.extract<bool>());
      case RosMsgParser::BuiltinType::STRING:
        return msgpack::pack_string(data, number.convert<std::string>());
      default:
        // fallback to int64 for all the other types
        return msgpack::pack_int(data, number.convert<int64_t>());
    }
  };

  std::string key_str;

  // Write numerical values as key-value pairs
  for (const auto& [key, num_value] : flat_msg.value) {
    key.toStr(key_str);
    resize_if_needed(4 + key_str.size() + 9);  // max key + max value size
    data_ptr += msgpack::pack_string(data_ptr, key_str);
    data_ptr += pack_variant(num_value, data_ptr);
  }

  msgpack_data.resize(static_cast<int>(data_ptr - msgpack_data.data()));
}
}  // namespace details

namespace nb = nanobind;

class Parser {
 private:
  RosMsgParser::Parser parser_;
  RosMsgParser::FlatMessage flat_msg_;
  RosMsgParser::NanoCDR_Deserializer deserializer_;
  std::vector<uint8_t> output_buffer_;

 public:
  Parser(const std::string& topic_name, const std::string& type_name, const std::string& schema)
      : parser_(topic_name, type_name, schema),
        output_buffer_(1024 * 1024)  // 1MB initial buffer
  {}

  ~Parser() = default;

  // Disable copy
  Parser(const Parser&) = delete;
  Parser& operator=(const Parser&) = delete;

  // Enable move
  Parser(Parser&& other) noexcept : parser_(std::move(other.parser_)) {}

  nb::bytes parse_to_msgpack(nb::bytes raw_data) {
    // Create span from input data
    RosMsgParser::Span<const uint8_t> msg_span(reinterpret_cast<const uint8_t*>(raw_data.c_str()), raw_data.size());

    if (!parser_.deserialize(msg_span, &flat_msg_, &deserializer_)) {
      throw std::runtime_error("Failed to parse ROS message");
    }

    details::convertToMsgpack(flat_msg_, output_buffer_);
    return nb::bytes(reinterpret_cast<const char*>(output_buffer_.data()), output_buffer_.size());
  }
};

NB_MODULE(rosx_introspection, m) {
  m.doc() = "Python bindings for rosx_introspection - ROS message parser";

  nb::class_<Parser>(m, "Parser")
      .def(
          nb::init<const std::string&, const std::string&, const std::string&>(), nb::arg("topic_name"),
          nb::arg("type_name"), nb::arg("schema"),
          "Create a ROS message parser.\n\n"
          "Args:\n"
          "    topic_name: Optional topic name (used as prefix in output)\n"
          "    type_name: ROS message type name\n"
          "    schema: ROS message schema definition")

      .def(
          "parse_to_msgpack", &Parser::parse_to_msgpack, nb::arg("raw_data"),
          "Parse raw ROS message to msgpack format.\n\n"
          "Args:\n"
          "    raw_data: Raw binary ROS message data\n\n"
          "Returns:\n"
          "    bytes: Msgpack-encoded parsed message");
}
