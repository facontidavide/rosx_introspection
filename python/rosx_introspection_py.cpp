#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <rosx_introspection/msgpack_utils.hpp>
#include <rosx_introspection/ros_parser.hpp>
#include <stdexcept>
#include <vector>

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

    RosMsgParser::convertToMsgpack(flat_msg_, output_buffer_);
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
