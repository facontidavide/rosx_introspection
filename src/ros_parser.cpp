/***** MIT License ****
 *
 *   Copyright (c) 2016-2024 Davide Faconti
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include "rosx_introspection/ros_parser.hpp"

#include <functional>
#include <limits>
#include <type_traits>

#ifdef ROSX_HAS_JSON
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rosx_introspection/deserializer.hpp"
#endif

namespace RosMsgParser {
inline bool operator==(const std::string& a, const std::string_view& b) {
  return (a.size() == b.size() && std::strncmp(a.data(), b.data(), a.size()) == 0);
}

Parser::Parser(const std::string& topic_name, const ROSType& msg_type, const std::string& definition,
               SchemaFormat format)
    : _global_warnings(&std::cerr),
      _topic_name(topic_name),
      _discard_large_array(DISCARD_LARGE_ARRAYS),
      _max_array_size(100),
      _blob_policy(STORE_BLOB_AS_COPY),
      _dummy_root_field(new ROSField(msg_type, topic_name)) {
  if (format == DDS_IDL) {
    _schema = ParseIDL(topic_name, msg_type, definition);
  } else {
    auto parsed_msgs = ParseMessageDefinitions(definition, msg_type);
    _schema = BuildMessageSchema(topic_name, parsed_msgs);
  }
}

const std::shared_ptr<MessageSchema>& Parser::getSchema() const {
  return _schema;
}

ROSMessage::Ptr Parser::getMessageByType(const ROSType& type) const {
  for (const auto& [msg_type, msg] : _schema->msg_library) {
    if (msg_type == type) {
      return msg;
    }
  }
  return {};
}

template <typename Container>
inline void ExpandVectorIfNecessary(Container& container, size_t new_size) {
  if (container.size() <= new_size) {
    const size_t increased_size = std::max(size_t(32), container.size() * 2);
    container.resize(increased_size);
  }
}

//=============================================================================
// Unified schema walk: walkSchema()
//=============================================================================

bool Parser::walkSchema(Span<const uint8_t> buffer, Deserializer* deserializer, SchemaWriter* writer) const {
  deserializer->init(buffer);
  bool entire_message_parsed = true;

  std::function<void(const ROSMessage*, FieldLeaf, bool)> walkImpl;

  walkImpl = [&](const ROSMessage* msg, FieldLeaf tree_leaf, bool store) {
    size_t index_s = 0;

    // --- @key fields: process first, add path suffixes ---
    // (ported from dds-parser: keys are deserialized before other fields
    //  and their values become path suffixes like [ArmID:3])
    for (size_t fi = 0; fi < msg->fields().size(); fi++) {
      const ROSField& field = msg->field(fi);
      if (field.isConstant()) {
        continue;
      }
      if (!field.isKey()) {
        continue;
      }

      if (field.type().typeID() == STRING) {
        std::string str;
        deserializer->deserializeString(str);
        tree_leaf.key_suffix = "[" + str + "]";
      } else if (field.getEnum() != nullptr) {
        Variant var = deserializer->deserialize(INT32);
        int32_t enum_int = var.convert<int32_t>();
        std::string enum_name = std::to_string(enum_int);
        for (const auto& ev : field.getEnum()->values) {
          if (ev.value == enum_int) {
            enum_name = ev.name;
            break;
          }
        }
        tree_leaf.key_suffix = "[" + enum_name + "]";
      } else if (field.type().isBuiltin()) {
        Variant var = deserializer->deserialize(field.type().typeID());
        tree_leaf.key_suffix =
            "[" + field.name() + ":" + std::to_string(var.convert<int64_t>()) + "]";
      }
    }

    // --- Process non-key fields ---
    for (const ROSField& field : msg->fields()) {
      bool DO_STORE = store;
      if (field.isConstant()) {
        continue;
      }
      if (field.isKey()) {
        index_s++;
        continue;  // already processed above
      }

      const ROSType& field_type = field.type();

      // Handle @optional fields
      if (field.isOptional()) {
        bool is_present = deserializer->hasOptionalMember();
        if (!is_present) {
          index_s++;
          continue;
        }
      }

      auto new_tree_leaf = tree_leaf;
      new_tree_leaf.node = tree_leaf.node->child(index_s);

      int32_t array_size = field.arraySize();
      if (array_size == -1) {
        array_size = deserializer->deserializeUInt32();
      }
      if (field.isArray()) {
        new_tree_leaf.index_array.push_back(0);
      }

      bool IS_BLOB = false;

      if (array_size > static_cast<int32_t>(_max_array_size)) {
        if (builtinSize(field_type.typeID()) == 1) {
          IS_BLOB = true;
        } else {
          if (_discard_large_array) {
            DO_STORE = false;
          }
          entire_message_parsed = false;
        }
      }

      if (IS_BLOB) {
        if (array_size > static_cast<int32_t>(deserializer->bytesLeft())) {
          throw std::runtime_error("Buffer overrun in walkSchema (blob)");
        }
        if (DO_STORE) {
          writer->writeBlob(new_tree_leaf, Span<const uint8_t>(deserializer->getCurrentPtr(), array_size));
        }
        deserializer->jump(array_size);
      } else {
        bool DO_STORE_ARRAY = DO_STORE;
        for (int i = 0; i < array_size; i++) {
          if (DO_STORE_ARRAY && i >= static_cast<int32_t>(_max_array_size)) {
            DO_STORE_ARRAY = false;
          }
          if (field.isArray() && DO_STORE_ARRAY) {
            new_tree_leaf.index_array.back() = i;
          }

          // --- Enum fields ---
          if (field.getEnum() != nullptr) {
            Variant var = deserializer->deserialize(INT32);
            if (DO_STORE_ARRAY) {
              int32_t enum_int = var.convert<int32_t>();
              std::string enum_name;
              for (const auto& ev : field.getEnum()->values) {
                if (ev.value == enum_int) {
                  enum_name = ev.name;
                  break;
                }
              }
              writer->writeEnum(new_tree_leaf, enum_int, enum_name);
            }
          }
          // --- Union fields ---
          else if (field.getUnion() != nullptr) {
            const auto* union_def = field.getUnion();
            auto disc_type = toBuiltinType(union_def->discriminant_type);
            std::string disc_value_str;

            if (disc_type == OTHER) {
              Variant disc_var = deserializer->deserialize(INT32);
              int32_t disc_int = disc_var.convert<int32_t>();
              auto enum_it = _schema->enum_library.find(ROSType(union_def->discriminant_type));
              if (enum_it != _schema->enum_library.end()) {
                for (const auto& ev : enum_it->second.values) {
                  if (ev.value == disc_int) {
                    disc_value_str = ev.name;
                    break;
                  }
                }
              }
              if (disc_value_str.empty()) {
                disc_value_str = std::to_string(disc_int);
              }
            } else {
              Variant disc_var = deserializer->deserialize(disc_type);
              disc_value_str = std::to_string(disc_var.convert<int64_t>());
            }

            auto case_it = union_def->cases.find(disc_value_str);
            const UnionCaseField* active_case = nullptr;
            if (case_it != union_def->cases.end()) {
              active_case = &case_it->second;
            } else if (union_def->default_case) {
              active_case = &union_def->default_case.value();
            }

            if (active_case) {
              if (active_case->type.typeID() == STRING) {
                std::string str;
                deserializer->deserializeString(str);
                if (DO_STORE_ARRAY) {
                  writer->writeString(new_tree_leaf, str);
                }
              } else if (active_case->type.isBuiltin()) {
                Variant var = deserializer->deserialize(active_case->type.typeID());
                if (DO_STORE_ARRAY) {
                  writer->writeValue(new_tree_leaf, var);
                }
              } else {
                auto case_msg_it = _schema->msg_library.find(active_case->type);
                if (case_msg_it != _schema->msg_library.end()) {
                  writer->beginStruct(field);
                  walkImpl(case_msg_it->second.get(), new_tree_leaf, DO_STORE_ARRAY);
                  writer->endStruct();
                }
              }
            }
          }
          // --- String fields ---
          else if (field_type.typeID() == STRING) {
            std::string str;
            deserializer->deserializeString(str);
            if (DO_STORE_ARRAY) {
              writer->writeString(new_tree_leaf, str);
            }
          }
          // --- Builtin scalar fields ---
          else if (field_type.isBuiltin()) {
            Variant var = deserializer->deserialize(field_type.typeID());
            if (DO_STORE_ARRAY) {
              writer->writeValue(new_tree_leaf, var);
            }
          }
          // --- Nested struct fields ---
          else {
            auto msg_node = field.getMessagePtr(_schema->msg_library);
            writer->beginStruct(field);
            walkImpl(msg_node.get(), new_tree_leaf, DO_STORE_ARRAY);
            writer->endStruct();
          }
        }  // end for array_size
      }

      index_s++;
    }  // end for fields
  };

  FieldLeaf rootnode;
  rootnode.node = _schema->field_tree.croot();
  auto root_msg = _schema->field_tree.croot()->value()->getMessagePtr(_schema->msg_library);

  walkImpl(root_msg.get(), rootnode, true);
  writer->finish();

  return entire_message_parsed;
}

//=============================================================================
// FlatMessageWriter: produces FlatMessage output
//=============================================================================

namespace {

class FlatMessageWriter : public SchemaWriter {
 public:
  FlatMessageWriter(FlatMessage* flat, Parser::BlobPolicy blob_policy)
      : _flat(flat), _blob_policy(blob_policy) {}

  void writeValue(const FieldLeaf& leaf, const Variant& value) override {
    ExpandVectorIfNecessary(_flat->value, _value_index);
    _flat->value[_value_index++] = {FieldsVector(leaf), value};
  }

  void writeString(const FieldLeaf& leaf, const std::string& str) override {
    ExpandVectorIfNecessary(_flat->value, _value_index);
    _flat->value[_value_index].first = FieldsVector(leaf);
    _flat->value[_value_index].second = str;
    _value_index++;
  }

  void writeEnum(const FieldLeaf& leaf, int32_t value, const std::string& /*name*/) override {
    writeValue(leaf, Variant(value));
  }

  void writeBlob(const FieldLeaf& leaf, Span<const uint8_t> data) override {
    ExpandVectorIfNecessary(_flat->blob, _blob_index);
    _flat->blob[_blob_index].first = FieldsVector(leaf);

    if (_blob_policy == Parser::STORE_BLOB_AS_COPY) {
      ExpandVectorIfNecessary(_flat->blob_storage, _blob_storage_index);
      auto& storage = _flat->blob_storage[_blob_storage_index];
      storage.assign(data.data(), data.data() + data.size());
      _flat->blob[_blob_index].second = Span<const uint8_t>(storage.data(), storage.size());
      _blob_storage_index++;
    } else {
      _flat->blob[_blob_index].second = data;
    }
    _blob_index++;
  }

  void finish() override {
    _flat->value.resize(_value_index);
    _flat->blob.resize(_blob_index);
    _flat->blob_storage.resize(_blob_storage_index);
  }

 private:
  FlatMessage* _flat;
  Parser::BlobPolicy _blob_policy;
  size_t _value_index = 0;
  size_t _blob_index = 0;
  size_t _blob_storage_index = 0;
};

}  // namespace

bool Parser::deserialize(Span<const uint8_t> buffer, FlatMessage* flat_container, Deserializer* deserializer) const {
  flat_container->schema = _schema;
  FlatMessageWriter writer(flat_container, _blob_policy);
  return walkSchema(buffer, deserializer, &writer);
}

//=============================================================================
// JSON support
//=============================================================================

#ifndef ROSX_HAS_JSON
bool Parser::deserializeIntoJson(
    Span<const uint8_t> buffer, std::string* json_txt, Deserializer* deserializer, int indent,
    bool ignore_constants) const {
  throw std::runtime_error("This version of rosx_introspection was built without JSON support");
  return false;
}
bool Parser::serializeFromJson(const std::string_view json_string, Serializer* serializer) const {
  throw std::runtime_error("This version of rosx_introspection was built without JSON support");
  return false;
}

#else

//-----------------------------------------------------------------------------
// JsonSchemaWriter: produces a rapidjson Document
//-----------------------------------------------------------------------------

namespace {

class JsonSchemaWriter : public SchemaWriter {
 public:
  JsonSchemaWriter(rapidjson::Document& doc, bool ignore_constants)
      : _doc(doc), _alloc(doc.GetAllocator()), _ignore_constants(ignore_constants) {
    _doc.SetObject();
    _json_stack.push_back(&_doc);
  }

  void writeValue(const FieldLeaf& leaf, const Variant& value) override {
    rapidjson::Value json_val;
    switch (value.getTypeID()) {
      case BOOL:
        json_val.SetBool(value.convert<uint8_t>());
        break;
      case CHAR: {
        char c = value.convert<int8_t>();
        json_val.SetString(&c, 1, _alloc);
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
      case DURATION: {
        // These are handled as two INT32s by the walker, arriving as individual fields
        json_val.SetInt(value.convert<int32_t>());
      } break;
      default:
        json_val.SetNull();
        break;
    }
    addToCurrentJson(leaf, std::move(json_val));
  }

  void writeString(const FieldLeaf& leaf, const std::string& str) override {
    rapidjson::Value json_val;
    json_val.SetString(str.c_str(), str.length(), _alloc);
    addToCurrentJson(leaf, std::move(json_val));
  }

  void writeEnum(const FieldLeaf& leaf, int32_t value, const std::string& /*name*/) override {
    rapidjson::Value json_val;
    json_val.SetInt(value);
    addToCurrentJson(leaf, std::move(json_val));
  }

  void beginStruct(const ROSField& /*field*/) override {
    // A new JSON object will be created by the walker when it recurses
  }

  void endStruct() override {
    // Nothing to do — the struct's fields are added directly to the current JSON level
  }

 private:
  void addToCurrentJson(const FieldLeaf& leaf, rapidjson::Value&& val) {
    // For now, the JsonSchemaWriter is a simplified version that stores
    // all values flat. The hierarchical JSON building requires the walker
    // to provide field names, which FieldLeaf already contains via the tree node.
    // We store in a flat structure for now.
    // Full hierarchical JSON would need more structural events from the walker.
    (void)leaf;
    (void)val;
    // TODO: build hierarchical JSON. For now, fall back to the legacy implementation.
  }

  rapidjson::Document& _doc;
  rapidjson::Document::AllocatorType& _alloc;
  bool _ignore_constants;
  std::vector<rapidjson::Value*> _json_stack;
};

}  // namespace

// For now, deserializeIntoJson uses the legacy implementation with IDL support added.
// A full JsonSchemaWriter that builds hierarchical JSON from the walkSchema events
// is planned as a follow-up.
bool Parser::deserializeIntoJson(
    Span<const uint8_t> buffer, std::string* json_txt, Deserializer* deserializer, int indent,
    bool ignore_constants) const {
  deserializer->init(buffer);

  rapidjson::Document json_document;
  rapidjson::Document::AllocatorType& alloc = json_document.GetAllocator();

  std::function<void(const ROSMessage*, rapidjson::Value&)> deserializeImpl;

  deserializeImpl = [&](const ROSMessage* msg_node, rapidjson::Value& json_value) {
    for (const ROSField& field : msg_node->fields()) {
      if (field.isConstant() && ignore_constants) {
        continue;
      }
      // Note: @key fields are included in JSON output as regular fields.
      // In FlatMessage they become path suffixes, but JSON has no path concept.

      const ROSType& field_type = field.type();
      auto field_name = rapidjson::StringRef(field.name().data(), field.name().size());

      // Handle @optional fields
      if (field.isOptional()) {
        bool is_present = deserializer->hasOptionalMember();
        if (!is_present) {
          continue;
        }
      }

      int32_t array_size = field.arraySize();
      if (array_size == -1) {
        array_size = deserializer->deserializeUInt32();
      }

      const bool skip_large_byte_array =
          field.isArray() && array_size > static_cast<int32_t>(_max_array_size) && field_type.isBuiltin() &&
          builtinSize(field_type.typeID()) == 1;

      if (skip_large_byte_array) {
        if (array_size > static_cast<int32_t>(deserializer->bytesLeft())) {
          throw std::runtime_error("Buffer overrun in blob");
        }
        deserializer->jump(array_size);
      } else {
        rapidjson::Value array_value(rapidjson::kArrayType);

        for (int i = 0; i < array_size; i++) {
          rapidjson::Value new_value;
          new_value.SetObject();

          // Handle enum fields
          if (field.getEnum() != nullptr) {
            new_value.SetInt(deserializer->deserialize(INT32).convert<int32_t>());
          }
          // Handle union fields
          else if (field.getUnion() != nullptr) {
            const auto* union_def = field.getUnion();
            auto disc_type = toBuiltinType(union_def->discriminant_type);
            std::string disc_value_str;

            if (disc_type == OTHER) {
              Variant disc_var = deserializer->deserialize(INT32);
              int32_t disc_int = disc_var.convert<int32_t>();
              auto enum_it = _schema->enum_library.find(ROSType(union_def->discriminant_type));
              if (enum_it != _schema->enum_library.end()) {
                for (const auto& ev : enum_it->second.values) {
                  if (ev.value == disc_int) {
                    disc_value_str = ev.name;
                    break;
                  }
                }
              }
              if (disc_value_str.empty()) {
                disc_value_str = std::to_string(disc_int);
              }
            } else {
              Variant disc_var = deserializer->deserialize(disc_type);
              disc_value_str = std::to_string(disc_var.convert<int64_t>());
            }

            auto case_it = union_def->cases.find(disc_value_str);
            const UnionCaseField* active_case = nullptr;
            if (case_it != union_def->cases.end()) {
              active_case = &case_it->second;
            } else if (union_def->default_case) {
              active_case = &union_def->default_case.value();
            }

            if (active_case) {
              if (active_case->type.typeID() == STRING) {
                std::string s;
                deserializer->deserializeString(s);
                new_value.SetString(s.c_str(), s.length(), alloc);
              } else if (active_case->type.isBuiltin()) {
                auto v = deserializer->deserialize(active_case->type.typeID());
                switch (active_case->type.typeID()) {
                  case FLOAT32:
                    new_value.SetFloat(v.convert<float>());
                    break;
                  case FLOAT64:
                    new_value.SetDouble(v.convert<double>());
                    break;
                  case UINT64:
                    new_value.SetUint64(v.convert<uint64_t>());
                    break;
                  case INT64:
                    new_value.SetInt64(v.convert<int64_t>());
                    break;
                  default:
                    new_value.SetInt(v.convert<int32_t>());
                    break;
                }
              } else {
                auto case_msg_it = _schema->msg_library.find(active_case->type);
                if (case_msg_it != _schema->msg_library.end()) {
                  deserializeImpl(case_msg_it->second.get(), new_value);
                }
              }
            }
          } else {
            // Standard field handling
            switch (field_type.typeID()) {
              case BOOL:
                new_value.SetBool(deserializer->deserialize(field_type.typeID()).convert<uint8_t>());
                break;
              case CHAR: {
                char c = deserializer->deserialize(field_type.typeID()).convert<int8_t>();
                new_value.SetString(&c, 1, alloc);
              } break;
              case BYTE:
              case UINT8:
              case UINT16:
              case UINT32:
                new_value.SetUint(deserializer->deserialize(field_type.typeID()).convert<uint32_t>());
                break;
              case UINT64:
                new_value.SetUint64(deserializer->deserialize(field_type.typeID()).convert<uint64_t>());
                break;
              case INT8:
              case INT16:
              case INT32:
                new_value.SetInt(deserializer->deserialize(field_type.typeID()).convert<int32_t>());
                break;
              case INT64:
                new_value.SetInt64(deserializer->deserialize(field_type.typeID()).convert<int64_t>());
                break;
              case FLOAT32:
                new_value.SetFloat(deserializer->deserialize(field_type.typeID()).convert<float>());
                break;
              case FLOAT64:
                new_value.SetDouble(deserializer->deserialize(field_type.typeID()).convert<double>());
                break;
              case TIME:
              case DURATION: {
                int sec = deserializer->deserialize(INT32).convert<int32_t>();
                int nsec = deserializer->deserialize(INT32).convert<int32_t>();
                rapidjson::Value sec_val;
                sec_val.SetInt(sec);
                new_value.AddMember("secs", sec_val, alloc);
                rapidjson::Value nsec_val;
                nsec_val.SetInt(nsec);
                new_value.AddMember("nsecs", nsec_val, alloc);
              } break;
              case STRING: {
                std::string s;
                deserializer->deserializeString(s);
                new_value.SetString(s.c_str(), s.length(), alloc);
              } break;
              case OTHER: {
                auto msg_node_child = field.getMessagePtr(_schema->msg_library);
                deserializeImpl(msg_node_child.get(), new_value);
              } break;
            }
          }

          if (field.isArray()) {
            array_value.PushBack(new_value, alloc);
          } else {
            json_value.AddMember(field_name, new_value, alloc);
          }
        }

        if (field.isArray()) {
          json_value.AddMember(field_name, array_value, alloc);
        }
      }
    }
  };

  rapidjson::Value& json_node = json_document.SetObject();
  auto root_msg = _schema->field_tree.croot()->value()->getMessagePtr(_schema->msg_library);
  deserializeImpl(root_msg.get(), json_node);

  thread_local rapidjson::StringBuffer json_buffer;
  json_buffer.Reserve(65536);
  json_buffer.Clear();

  if (indent == 0) {
    rapidjson::Writer<
        rapidjson::StringBuffer, rapidjson::UTF8<>, rapidjson::UTF8<>, rapidjson::CrtAllocator,
        rapidjson::kWriteDefaultFlags | rapidjson::kWriteNanAndInfFlag>
        json_writer(json_buffer);
    json_document.Accept(json_writer);
    *json_txt = json_buffer.GetString();
  } else {
    rapidjson::PrettyWriter<rapidjson::StringBuffer> json_writer(json_buffer);
    json_writer.SetIndent(' ', indent);
    json_document.Accept(json_writer);
    *json_txt = json_buffer.GetString();
  }

  return true;
}

//-----------------------------------------------------------------------------
// serializeFromJson
//-----------------------------------------------------------------------------

namespace {
template <typename T>
T readJsonValue(rapidjson::Value* value_field, std::string_view field_name) {
  if (!value_field) {
    return T{};
  }
  if constexpr (std::is_unsigned_v<T> && std::is_integral_v<T>) {
    if (!value_field->IsUint64()) {
      throw std::runtime_error("Expected unsigned integer in field: " + std::string(field_name));
    }
    auto v = value_field->GetUint64();
    if (v > std::numeric_limits<T>::max()) {
      throw std::runtime_error("Value out of range in field: " + std::string(field_name));
    }
    return static_cast<T>(v);
  } else if constexpr (std::is_signed_v<T> && std::is_integral_v<T>) {
    if (!value_field->IsInt64()) {
      throw std::runtime_error("Expected integer in field: " + std::string(field_name));
    }
    auto v = value_field->GetInt64();
    if (v < std::numeric_limits<T>::min() || v > std::numeric_limits<T>::max()) {
      throw std::runtime_error("Value out of range in field: " + std::string(field_name));
    }
    return static_cast<T>(v);
  } else {
    static_assert(std::is_floating_point_v<T>);
    if (!value_field->IsNumber()) {
      throw std::runtime_error("Expected number in field: " + std::string(field_name));
    }
    return static_cast<T>(value_field->GetDouble());
  }
}
}  // namespace

bool Parser::serializeFromJson(const std::string_view json_string, Serializer* serializer) const {
  rapidjson::Document json_document;
  json_document.Parse(json_string.data(), json_string.size());
  if (json_document.HasParseError()) {
    throw std::runtime_error("Failed to parse JSON input");
  }
  if (!json_document.IsObject()) {
    throw std::runtime_error("JSON root must be an object");
  }
  serializer->reset();

  std::function<void(const ROSMessage*, rapidjson::Value*)> serializeImpl;

  serializeImpl = [&](const ROSMessage* msg_node, rapidjson::Value* json_value) {
    if (json_value && !json_value->IsObject()) {
      throw std::runtime_error("Expected JSON object while serializing nested message");
    }

    for (const ROSField& field : msg_node->fields()) {
      if (field.isConstant()) {
        continue;
      }
      const ROSType& field_type = field.type();
      const auto field_name = rapidjson::StringRef(field.name().data(), field.name().size());

      // Handle enum fields: serialize as INT32
      if (field.getEnum() != nullptr) {
        rapidjson::Value* field_json_value = nullptr;
        if (json_value && json_value->HasMember(field_name.s)) {
          field_json_value = &((*json_value)[field_name.s]);
        }
        int32_t val = field_json_value ? readJsonValue<int32_t>(field_json_value, field.name()) : 0;
        serializer->serialize(INT32, Variant(val));
        continue;
      }

      // Handle union fields: not supported in JSON serialization
      if (field.getUnion() != nullptr) {
        throw std::runtime_error("serializeFromJson not supported for union fields: " + field.name());
      }

      rapidjson::Value* field_json_value = nullptr;
      if (json_value && json_value->HasMember(field_name.s)) {
        field_json_value = &((*json_value)[field_name.s]);
      }

      const bool has_json_value = (field_json_value != nullptr);
      const bool is_array = field.isArray();
      const bool is_dynamic_array = is_array && field.arraySize() == -1;
      const bool is_fixed_array = is_array && field.arraySize() != -1;

      if (has_json_value && (is_array != field_json_value->IsArray())) {
        throw std::runtime_error(std::string("IsArray() mismatch in field: ") + field.name());
      }

      uint32_t array_size = field.arraySize();
      if (is_dynamic_array) {
        array_size = has_json_value ? static_cast<uint32_t>(field_json_value->GetArray().Size()) : 0;
        serializer->serializeUInt32(array_size);
      }
      if (has_json_value && is_fixed_array) {
        auto actual_size = static_cast<uint32_t>(field_json_value->GetArray().Size());
        if (array_size != actual_size) {
          throw std::runtime_error(std::string("Fixed array size mismatch in field: ") + field.name());
        }
      }

      const auto type_id = field_type.typeID();
      const auto fname = field.name();

      for (uint32_t i = 0; i < array_size; i++) {
        rapidjson::Value* value_field = nullptr;
        if (has_json_value) {
          value_field = is_array ? &(field_json_value->GetArray()[i]) : field_json_value;
        }

        switch (type_id) {
          case BOOL: {
            bool value = value_field ? value_field->GetBool() : false;
            serializer->serialize(type_id, value);
          } break;
          case CHAR: {
            char c = '\0';
            if (value_field && value_field->IsString() && value_field->GetStringLength() > 0) {
              c = value_field->GetString()[0];
            }
            serializer->serialize(type_id, c);
          } break;
          case BYTE:
          case UINT8:
            serializer->serialize(type_id, readJsonValue<uint8_t>(value_field, fname));
            break;
          case UINT16:
            serializer->serialize(type_id, readJsonValue<uint16_t>(value_field, fname));
            break;
          case UINT32:
            serializer->serialize(type_id, readJsonValue<uint32_t>(value_field, fname));
            break;
          case UINT64:
            serializer->serialize(type_id, readJsonValue<uint64_t>(value_field, fname));
            break;
          case INT8:
            serializer->serialize(type_id, readJsonValue<int8_t>(value_field, fname));
            break;
          case INT16:
            serializer->serialize(type_id, readJsonValue<int16_t>(value_field, fname));
            break;
          case INT32:
            serializer->serialize(type_id, readJsonValue<int32_t>(value_field, fname));
            break;
          case INT64:
            serializer->serialize(type_id, readJsonValue<int64_t>(value_field, fname));
            break;
          case FLOAT32:
            serializer->serialize(type_id, readJsonValue<float>(value_field, fname));
            break;
          case FLOAT64:
            serializer->serialize(type_id, readJsonValue<double>(value_field, fname));
            break;
          case DURATION:
          case TIME: {
            int32_t secs = 0;
            int32_t nsecs = 0;
            if (value_field) {
              if (!value_field->IsObject()) {
                throw std::runtime_error(std::string("Expected time/duration object in field: ") + fname);
              }
              auto sec_it = value_field->FindMember("secs");
              auto nsec_it = value_field->FindMember("nsecs");
              if (sec_it == value_field->MemberEnd() || nsec_it == value_field->MemberEnd()) {
                throw std::runtime_error(std::string("Missing secs/nsecs in field: ") + fname);
              }
              secs = readJsonValue<int32_t>(&sec_it->value, fname);
              nsecs = readJsonValue<int32_t>(&nsec_it->value, fname);
            }
            serializer->serializeUInt32(static_cast<uint32_t>(secs));
            serializer->serializeUInt32(static_cast<uint32_t>(nsecs));
          } break;
          case STRING: {
            if (value_field) {
              const char* str = value_field->GetString();
              uint32_t len = value_field->GetStringLength();
              serializer->serializeString(std::string(str, len));
            } else {
              serializer->serializeString("");
            }
          } break;
          case OTHER: {
            auto msg_node_child = field.getMessagePtr(_schema->msg_library);
            if (!msg_node_child) {
              throw std::runtime_error(std::string("Missing ROSType in library for field: ") + fname);
            }
            serializeImpl(msg_node_child.get(), value_field);
          } break;
        }
      }
    }
  };

  auto root_msg = _schema->field_tree.croot()->value()->getMessagePtr(_schema->msg_library);
  rapidjson::Value& json_root = json_document;
  serializeImpl(root_msg.get(), &json_root);

  return true;
}

#endif

//=============================================================================
// applyVisitorToBuffer (unchanged from original)
//=============================================================================

void Parser::applyVisitorToBuffer(const ROSType& /*msg_type*/, Span<uint8_t>& /*buffer*/,
                                  VisitingCallback /*callback*/) const {
  throw std::runtime_error("applyVisitorToBuffer is not implemented");
}

}  // namespace RosMsgParser
