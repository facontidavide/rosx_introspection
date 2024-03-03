/***** MIT License ****
 *
 *   Copyright (c) 2016-2022 Davide Faconti
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

#include <functional>

#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/deserializer.hpp"

namespace RosMsgParser
{
  inline bool operator==(const std::string& a, const std::string_view& b)
  {
    return (a.size() == b.size() && std::strncmp(a.data(), b.data(), a.size()) == 0);
  }

  Parser::Parser(const std::string& topic_name, const ROSType& msg_type,
                 const std::string& definition)
      : _global_warnings(&std::cerr)
      , _topic_name(topic_name)
      , _discard_large_array(DISCARD_LARGE_ARRAYS)
      , _max_array_size(100)
      , _blob_policy(STORE_BLOB_AS_COPY)
      , _dummy_root_field(new ROSField(msg_type, topic_name))
  {
    auto parsed_msgs = ParseMessageDefinitions(definition, msg_type);
    _schema = BuildMessageSchema(topic_name, parsed_msgs);
  }

  const std::shared_ptr<MessageSchema>& Parser::getSchema() const
  {
    return _schema;
  }

  ROSMessage::Ptr Parser::getMessageByType(const ROSType& type) const
  {
    for (const auto& [msg_type, msg] : _schema->msg_library)  // find in the list
    {
      if (msg_type == type)
      {
        return msg;
      }
    }
    return {};
  }

  template <typename Container>
  inline void ExpandVectorIfNecessary(Container& container, size_t new_size)
  {
    if (container.size() <= new_size)
    {
      const size_t increased_size = std::max(size_t(32), container.size() * 2);
      container.resize(increased_size);
    }
  }

  bool Parser::deserializeIntoJson(Span<const uint8_t> buffer, std::string *json_txt, Deserializer *deserializer, bool ignore_constants) const
  {
    deserializer->init(buffer);

    rapidjson::Document json_document;
    rapidjson::Document::AllocatorType &alloc = json_document.GetAllocator();

    size_t buffer_offset = 0;

    std::function<void(const ROSMessage *, rapidjson::Value &)> deserializeImpl;

    deserializeImpl = [&](const ROSMessage *msg_node, rapidjson::Value &json_value)
    {
      // const ROSMessage *msg_definition = msg_node->value();
      size_t index_s = 0;
      size_t index_m = 0;

      for (const ROSField &field : msg_node->fields())
      {
        if (field.isConstant() && ignore_constants)
          continue;

        const ROSType &field_type = field.type();
        auto field_name = rapidjson::StringRef(field.name().data(), field.name().size());

        int32_t array_size = field.arraySize();
        if (array_size == -1)
        {
          array_size = deserializer->deserializeUInt32();

          // ReadFromBuffer(buffer, buffer_offset, array_size);
        }

        // Stop storing if it is a blob.
        if (array_size > static_cast<int32_t>(_max_array_size))
        {
          if (buffer_offset + array_size > static_cast<std::size_t>(buffer.size()))
          {
            throw std::runtime_error("Buffer overrun in blob");
          }
          buffer_offset += array_size;
        }
        else // NOT a BLOB
        {
          rapidjson::Value array_value(rapidjson::kArrayType);

          for (int i = 0; i < array_size; i++)
          {
            rapidjson::Value new_value;
            new_value.SetObject();

            switch (field_type.typeID())
            {
            case BOOL:
              // new_value.SetBool(ReadFromBuffer<bool>(buffer, buffer_offset));
              new_value.SetBool(deserializer->deserialize(field_type.typeID()).convert<uint8_t>());
              break;
            case CHAR:
            {
              char c = deserializer->deserialize(field_type.typeID()).convert<int8_t>();
              new_value.SetString(&c, 1, alloc);
            }
            break;
            case BYTE:
            case UINT8:
              new_value.SetUint((deserializer->deserialize(field_type.typeID()).convert<uint8_t>()));
              break;
            case UINT16:
              new_value.SetUint((deserializer->deserialize(field_type.typeID()).convert<uint16_t>()));
              break;
            case UINT32:
              new_value.SetUint((deserializer->deserialize(field_type.typeID()).convert<uint32_t>()));
              break;
            case UINT64:
              new_value.SetUint64((deserializer->deserialize(field_type.typeID()).convert<uint64_t>()));
              break;
            case INT8:
              new_value.SetInt((deserializer->deserialize(field_type.typeID()).convert<int8_t>()));
              break;
            case INT16:
              new_value.SetInt((deserializer->deserialize(field_type.typeID()).convert<int16_t>()));
              break;
            case INT32:
              new_value.SetInt((deserializer->deserialize(field_type.typeID()).convert<int32_t>()));
              break;
            case INT64:
              new_value.SetInt64((deserializer->deserialize(field_type.typeID()).convert<int64_t>()));
              break;
            case FLOAT32:
              new_value.SetFloat((deserializer->deserialize(field_type.typeID()).convert<float>()));
              break;
            case FLOAT64:
              new_value.SetDouble((deserializer->deserialize(field_type.typeID()).convert<double>()));
              break;
            case TIME:
            {
              int sec = deserializer->deserialize(INT32).convert<int32_t>();
              int nsec = deserializer->deserialize(INT32).convert<int32_t>();

              // ReadFromBuffer(buffer, buffer_offset, tmp.sec);
              // ReadFromBuffer(buffer, buffer_offset, tmp.nsec);

              rapidjson::Value sec_Value;
              sec_Value.SetObject();
              sec_Value.SetInt(sec);
              new_value.AddMember("secs", sec_Value, alloc);

              rapidjson::Value nsec_value;
              nsec_value.SetObject();
              nsec_value.SetInt(nsec);
              new_value.AddMember("nsecs", nsec_value, alloc);
            }
            break;
            case DURATION:
            {
              int sec = deserializer->deserialize(INT32).convert<int32_t>();
              int nsec = deserializer->deserialize(INT32).convert<int32_t>();

              // ReadFromBuffer(buffer, buffer_offset, tmp.sec);
              // ReadFromBuffer(buffer, buffer_offset, tmp.nsec);

              rapidjson::Value sec_Value;
              sec_Value.SetObject();
              sec_Value.SetInt(sec);
              new_value.AddMember("secs", sec_Value, alloc);

              rapidjson::Value nsec_value;
              nsec_value.SetObject();
              nsec_value.SetInt(nsec);
              new_value.AddMember("nsecs", nsec_value, alloc);
            }
            break;

            case STRING:
            {
              std::string s;
              deserializer->deserializeString(s);

              // uint32_t string_size = 0;
              // ReadFromBuffer(buffer, buffer_offset, string_size);
              // if (buffer_offset + string_size > static_cast<std::size_t>(buffer.size()))
              // {
              //   throw std::runtime_error("Buffer overrun");
              // }
              new_value.SetString(s.c_str(), s.length(), alloc);
              // buffer_offset += string_size;
            }
            break;
            case OTHER:
            {

              auto msg_node_child = field.getMessagePtr(_schema->msg_library);
              // deserializeImpl(msg_node.get(), new_tree_leaf, DO_STORE_ARRAY);

              deserializeImpl(msg_node_child.get(), new_value);
            }
            break;
            } // end switch

            if (field.isArray())
            {
              array_value.PushBack(new_value, alloc);
            }
            else
            {
              json_value.AddMember(field_name, new_value, alloc);
            }
          } // end for array

          if (field.isArray())
          {
            json_value.AddMember(field_name, array_value, alloc);
          }
        } // end for array_size

        if (field_type.typeID() == OTHER)
        {
          index_m++;
        }
        index_s++;
      } // end for fields
    };  // end of lambda

    // pass the shared_ptr

    FieldLeaf rootnode;
    rootnode.node = _schema->field_tree.croot();
    auto root_msg =
        _schema->field_tree.croot()->value()->getMessagePtr(_schema->msg_library);

    json_document.SetObject();
    rapidjson::Value json_node;
    json_node.SetObject();

    deserializeImpl(root_msg.get(), json_node);

    auto topic_name = rapidjson::StringRef(_topic_name.data(), _topic_name.size());
    json_document.AddMember("topic", topic_name, alloc);
    json_document.AddMember("msg", json_node, alloc);

    rapidjson::StringBuffer json_buffer;
    json_buffer.Reserve(2048);

    rapidjson::Writer<rapidjson::StringBuffer,
                      rapidjson::UTF8<>,
                      rapidjson::UTF8<>,
                      rapidjson::CrtAllocator,
                      rapidjson::kWriteDefaultFlags |
                          rapidjson::kWriteNanAndInfFlag>
        json_writer(json_buffer);
    json_document.Accept(json_writer);

    *json_txt = json_buffer.GetString();

    return true;
  }

  bool Parser::serializeFromJson(std::vector<uint8_t> &bufferOut, std::string *json_txt) const
  {
    rapidjson::Document json_document;
    json_document.Parse(json_txt->c_str());
    rapidjson::Document::AllocatorType &alloc = json_document.GetAllocator();

    uint8_t *buffer_data;

    size_t buffer_offset = 0;

    std::function<void(const ROSMessage *, rapidjson::Value &)> deserializeImpl;
    // rapidjson::Value new_value;

    deserializeImpl = [&](const ROSMessage *msg_node, rapidjson::Value &new_value)
    {
      // const ROSMessage *msg_definition = msg_node->value();
      size_t index_s = 0;
      size_t index_m = 0;

      for (const ROSField &field : msg_node->fields())
      {

        // if (field.isConstant())
        //   continue;

        const ROSType &field_type = field.type();
        auto field_name = rapidjson::StringRef(field.name().data(), field.name().size());

        uint32_t array_size = field.arraySize();

        if (array_size == -1)
        {
          if (!new_value.HasMember(field_name.s))
          {

            throw std::runtime_error("looks like it is a blob that wasn't serialized");
          }

          array_size = new_value[field_name.s].GetArray().Size();
          buffer_data = reinterpret_cast<uint8_t *>(&array_size);
          bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint32_t));
        }

        for (int i = 0; i < array_size; i++)
        {

          switch (field_type.typeID())
          {
          case BOOL:
          {
            bool val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetBool();
            }
            else
            {
              val = new_value[field_name.s].GetBool();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(bool));
          }
          break;
          case CHAR:
          {

            char val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetString()[0];
            }
            else
            {
              val = new_value[field_name.s].GetString()[0];
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(char));
          }
          break;
          case BYTE:
          case UINT8:
          {

            uint8_t val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetUint();
            }
            else
            {
              val = new_value[field_name.s].GetUint();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint8_t));
          }
          break;
          case UINT16:
          {
            uint16_t val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetUint();
            }
            else
            {
              val = new_value[field_name.s].GetUint();
            }

            buffer_data = reinterpret_cast<uint8_t *>(&val);

            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint16_t));
          }
          break;
          case UINT32:
          {

            uint32_t val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetUint();
            }
            else
            {
              val = new_value[field_name.s].GetUint();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint32_t));
          }
          break;
          case UINT64:
          {

            uint64_t val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetUint64();
            }
            else
            {
              val = new_value[field_name.s].GetUint64();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint64_t));
          }
          break;
          case INT8:
          {
            int8_t val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetInt();
            }
            else
            {
              val = new_value[field_name.s].GetInt();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(int8_t));
          }
          break;
          case INT16:
          {
            int16_t val;

            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetInt();
            }
            else
            {
              val = new_value[field_name.s].GetInt();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(int16_t));
          }
          break;
          case INT32:
          {
            int32_t val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetInt();
            }
            else
            {
              val = new_value[field_name.s].GetInt();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(int32_t));
          }
          break;
          case INT64:
          {
            int64_t val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetInt64();
            }
            else
            {
              val = new_value[field_name.s].GetInt64();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(int64_t));
          }
          break;
          case FLOAT32:
          {
            float val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetFloat();
            }
            else
            {
              val = new_value[field_name.s].GetFloat();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(float));
          }
          break;
          case FLOAT64:
          {
            double val;
            if (new_value[field_name.s].IsArray())
            {
              val = new_value[field_name.s].GetArray()[i].GetDouble();
            }
            else
            {
              val = new_value[field_name.s].GetDouble();
            }
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(double));
          }
          break;
          case TIME:
          {

            int val = new_value[field_name.s].GetObject()["secs"].GetInt();
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint32_t));
            
            val = new_value[field_name.s].GetObject()["nsecs"].GetInt();
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint32_t));
          }
          break;
          case DURATION:
          {

            int val = new_value[field_name.s].GetObject()["secs"].GetInt();
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint32_t));

            val = new_value[field_name.s].GetObject()["nsecs"].GetInt();
            buffer_data = reinterpret_cast<uint8_t *>(&val);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint32_t));
          }
          break;

          case STRING:
          {

            char *val;
            uint32_t len;

            if (new_value[field_name.s].IsArray())
            {
              val = (char *)new_value[field_name.s].GetArray()[i].GetString();
              len = new_value[field_name.s].GetArray()[i].GetStringLength();
            }
            else
            {
              val = (char *)new_value[field_name.s].GetString();
              len = new_value[field_name.s].GetStringLength();
            }

            buffer_data = reinterpret_cast<uint8_t *>(&len);
            bufferOut.insert(bufferOut.end(), buffer_data, buffer_data + sizeof(uint32_t));

            bufferOut.insert(bufferOut.end(), val, val + len);
          }
          break;
          case OTHER:
          {
            rapidjson::Value::Object new_value2 = new_value[field_name.s].GetObject();

            auto msg_node_child = field.getMessagePtr(_schema->msg_library);

            deserializeImpl(msg_node_child.get(), new_value2);
            // deserializeImpl(msg_node->child(index_m), new_value2);
          }
          break;
          } // end switch

        } // end for array

        if (field_type.typeID() == OTHER)
        {
          index_m++;
        }
        index_s++;
      } // end for fields
    };  // end of lambda

    // pass the shared_ptr

    rapidjson::Value json_node;
    json_node = json_document.GetObject();

    FieldLeaf rootnode;
    rootnode.node = _schema->field_tree.croot();
    auto root_msg =
        _schema->field_tree.croot()->value()->getMessagePtr(_schema->msg_library);

    deserializeImpl(root_msg.get(), json_node);
    // deserializeImpl(_message_info->message_tree.croot(), json_node);

    return true;
  }

  bool Parser::deserialize(Span<const uint8_t> buffer, FlatMessage* flat_container,
                           Deserializer* deserializer) const
  {
    deserializer->init(buffer);

    bool entire_message_parse = true;

    size_t value_index = 0;
    size_t name_index = 0;
    size_t blob_index = 0;
    size_t blob_storage_index = 0;

    std::function<void(const ROSMessage*, FieldLeaf, bool)> deserializeImpl;

    deserializeImpl = [&](const ROSMessage* msg, FieldLeaf tree_leaf, bool store) {
      size_t index_s = 0;
      size_t index_m = 0;

      for (const ROSField& field : msg->fields())
      {
        bool DO_STORE = store;
        if (field.isConstant())
        {
          continue;
        }

        const ROSType& field_type = field.type();

        auto new_tree_leaf = tree_leaf;
        new_tree_leaf.node = tree_leaf.node->child(index_s);

        int32_t array_size = field.arraySize();
        if (array_size == -1)
        {
          array_size = deserializer->deserializeUInt32();
        }
        if (field.isArray())
        {
          new_tree_leaf.index_array.push_back(0);
        }

        bool IS_BLOB = false;

        // Stop storing it if is NOT a blob and a very large array.
        if (array_size > static_cast<int32_t>(_max_array_size) &&
            field_type.typeID() == BuiltinType::OTHER)
        {
          if (builtinSize(field_type.typeID()) == 1)
          {
            IS_BLOB = true;
          }
          else
          {
            if (_discard_large_array)
            {
              DO_STORE = false;
            }
            entire_message_parse = false;
          }
        }

        if (IS_BLOB)  // special case. This is a "blob", typically an image, a map,
                     // pointcloud, etc.
        {
          ExpandVectorIfNecessary(flat_container->blob, blob_index);

          if (array_size > deserializer->bytesLeft())
          {
            throw std::runtime_error("Buffer overrun in deserializeIntoFlatContainer "
                                     "(blob)");
          }
          if (DO_STORE)
          {
            flat_container->blob[blob_index].first = FieldsVector(new_tree_leaf);
            auto& blob = flat_container->blob[blob_index].second;
            blob_index++;

            if (_blob_policy == STORE_BLOB_AS_COPY)
            {
              ExpandVectorIfNecessary(flat_container->blob_storage, blob_storage_index);

              auto& storage = flat_container->blob_storage[blob_storage_index];
              storage.resize(array_size);
              std::memcpy(storage.data(), deserializer->getCurrentPtr(), array_size);
              blob_storage_index++;

              blob = Span<const uint8_t>(storage.data(), storage.size());
            }
            else
            {
              blob = Span<const uint8_t>(deserializer->getCurrentPtr(), array_size);
            }
          }
          deserializer->jump(array_size);
        }
        else  // NOT a BLOB
        {
          bool DO_STORE_ARRAY = DO_STORE;
          for (int i = 0; i < array_size; i++)
          {
            if (DO_STORE_ARRAY && i >= static_cast<int32_t>(_max_array_size))
            {
              DO_STORE_ARRAY = false;
            }

            if (field.isArray() && DO_STORE_ARRAY)
            {
              new_tree_leaf.index_array.back() = i;
            }

            if (field_type.typeID() == STRING)
            {
              ExpandVectorIfNecessary(flat_container->name, name_index);

              std::string str;
              deserializer->deserializeString(str);

              if (DO_STORE_ARRAY)
              {
                flat_container->name[name_index].first = FieldsVector(new_tree_leaf);
                flat_container->name[name_index].second = str;
                name_index++;
              }
            }
            else if (field_type.isBuiltin())
            {
              ExpandVectorIfNecessary(flat_container->value, value_index);

              Variant var = deserializer->deserialize(field_type.typeID());
              if (DO_STORE_ARRAY)
              {
                flat_container->value[value_index] =
                    std::make_pair(new_tree_leaf, std::move(var));
                value_index++;
              }
            }
            else
            {  // field_type.typeID() == OTHER
              auto msg_node = field.getMessagePtr(_schema->msg_library);
              deserializeImpl(msg_node.get(), new_tree_leaf, DO_STORE_ARRAY);
            }
          }  // end for array_size
        }

        if (field_type.typeID() == OTHER)
        {
          index_m++;
        }
        index_s++;
      }  // end for fields
    };   // end of lambda

    // pass the shared_ptr
    flat_container->schema = _schema;

    FieldLeaf rootnode;
    rootnode.node = _schema->field_tree.croot();
    auto root_msg =
        _schema->field_tree.croot()->value()->getMessagePtr(_schema->msg_library);

    deserializeImpl(root_msg.get(), rootnode, true);

    flat_container->name.resize(name_index);
    flat_container->value.resize(value_index);
    flat_container->blob.resize(blob_index);
    flat_container->blob_storage.resize(blob_storage_index);

    return entire_message_parse;
  }


}  // namespace RosMsgParser
