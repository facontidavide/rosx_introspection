#pragma once

#include <memory>

#include "rosx_introspection/schema_writer.hpp"

#ifdef ROSX_HAS_JSON

namespace RosMsgParser {

class JsonSchemaWriter : public SchemaWriter {
 public:
  JsonSchemaWriter(void* doc, bool ignore_constants);
  ~JsonSchemaWriter() override;

  JsonSchemaWriter(const JsonSchemaWriter&) = delete;
  JsonSchemaWriter& operator=(const JsonSchemaWriter&) = delete;

  void writeValue(const FieldLeaf& leaf, const Variant& value) override;
  void writeString(const FieldLeaf& leaf, const std::string& str) override;
  void writeEnum(const FieldLeaf& leaf, int32_t value, const std::string& name) override;
  void beginStruct(const ROSField& field) override;
  void endStruct() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};

}  // namespace RosMsgParser

#endif  // ROSX_HAS_JSON
