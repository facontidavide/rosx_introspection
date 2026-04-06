#include "rosx_introspection/json_schema_writer.hpp"

#ifdef ROSX_HAS_JSON

#include "rapidjson/document.h"

#include <vector>

#include "rosx_introspection/ros_field.hpp"
#include "rosx_introspection/variant.hpp"

namespace RosMsgParser {

struct JsonSchemaWriter::Impl {
  rapidjson::Document& doc;
  rapidjson::Document::AllocatorType& alloc;
  bool ignore_constants;
  std::vector<rapidjson::Value*> json_stack;

  Impl(rapidjson::Document& d, bool ignore)
      : doc(d), alloc(d.GetAllocator()), ignore_constants(ignore) {
    doc.SetObject();
    json_stack.push_back(&doc);
  }

  void addToCurrentJson(const FieldLeaf& leaf, rapidjson::Value&& val) {
    (void)leaf;
    (void)val;
    // TODO: build hierarchical JSON. For now, fall back to the legacy implementation.
  }
};

JsonSchemaWriter::JsonSchemaWriter(void* doc, bool ignore_constants)
    : _impl(std::make_unique<Impl>(*static_cast<rapidjson::Document*>(doc), ignore_constants)) {}

JsonSchemaWriter::~JsonSchemaWriter() = default;

void JsonSchemaWriter::writeValue(const FieldLeaf& leaf, const Variant& value) {
  rapidjson::Value json_val;
  switch (value.getTypeID()) {
    case BOOL:
      json_val.SetBool(value.convert<uint8_t>());
      break;
    case CHAR: {
      char c = value.convert<int8_t>();
      json_val.SetString(&c, 1, _impl->alloc);
    } break;
    case BYTE:
    case UINT8:
    case UINT16:
    case UINT32:
      json_val.SetUint(value.convert<uint32_t>());
      break;
    case UINT64:
      json_val.SetUint64(value.convert<uint64_t>());
      break;
    case INT8:
    case INT16:
    case INT32:
      json_val.SetInt(value.convert<int32_t>());
      break;
    case INT64:
      json_val.SetInt64(value.convert<int64_t>());
      break;
    case FLOAT32:
      json_val.SetFloat(value.convert<float>());
      break;
    case FLOAT64:
      json_val.SetDouble(value.convert<double>());
      break;
    case TIME:
    case DURATION:
      json_val.SetInt(value.convert<int32_t>());
      break;
    default:
      json_val.SetNull();
      break;
  }
  _impl->addToCurrentJson(leaf, std::move(json_val));
}

void JsonSchemaWriter::writeString(const FieldLeaf& leaf, const std::string& str) {
  rapidjson::Value json_val;
  json_val.SetString(str.c_str(), str.length(), _impl->alloc);
  _impl->addToCurrentJson(leaf, std::move(json_val));
}

void JsonSchemaWriter::writeEnum(const FieldLeaf& leaf, int32_t value, const std::string& /*name*/) {
  rapidjson::Value json_val;
  json_val.SetInt(value);
  _impl->addToCurrentJson(leaf, std::move(json_val));
}

void JsonSchemaWriter::beginStruct(const ROSField& /*field*/) {}

void JsonSchemaWriter::endStruct() {}

}  // namespace RosMsgParser

#endif  // ROSX_HAS_JSON
