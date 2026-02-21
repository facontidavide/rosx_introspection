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

#include "rosx_introspection/msgpack_utils.hpp"
#include "rosx_introspection/contrib/msgpack.hpp"

#include <string>

namespace RosMsgParser {

void convertToMsgpack(const FlatMessage& flat_msg, std::vector<uint8_t>& msgpack_data) {
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

  auto pack_variant = [](const Variant& number, uint8_t*& data) -> uint32_t {
    switch (number.getTypeID()) {
      case BuiltinType::UINT64:
        return msgpack::pack_uint(data, number.extract<uint64_t>());
      case BuiltinType::FLOAT64:
        return msgpack::pack_double(data, number.extract<double>());
      case BuiltinType::FLOAT32:
        return msgpack::pack_float(data, number.extract<float>());
      case BuiltinType::BOOL:
        return msgpack::pack_bool(data, number.extract<bool>());
      case BuiltinType::STRING:
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

}  // namespace RosMsgParser
