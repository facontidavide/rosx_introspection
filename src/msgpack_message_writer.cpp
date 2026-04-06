#include "rosx_introspection/msgpack_message_writer.hpp"

#include "rosx_introspection/contrib/msgpack.hpp"

namespace RosMsgParser {

MsgpackMessageWriter::MsgpackMessageWriter(std::vector<uint8_t>* output) : _output(output) {
  _output->resize(64 * 1024);
  // Reserve 5 bytes for the map header (MAP32 worst case), written in finish()
  _offset = 5;
  _count = 0;
}

void MsgpackMessageWriter::ensureCapacity(size_t additional) {
  size_t required = _offset + additional;
  if (_output->size() >= required) {
    return;
  }
  size_t new_size = _output->size();
  while (new_size < required) {
    new_size *= 2;
  }
  _output->resize(new_size);
}

void MsgpackMessageWriter::writeKey(const FieldLeaf& leaf) {
  leaf.toStr(_key_buf);
  ensureCapacity(5 + _key_buf.size());
  _offset += msgpack::pack_string(_output->data() + _offset, _key_buf);
}

void MsgpackMessageWriter::writeValue(const FieldLeaf& leaf, const Variant& value) {
  writeKey(leaf);
  ensureCapacity(9);
  switch (value.getTypeID()) {
    case UINT64:
      _offset += msgpack::pack_uint(_output->data() + _offset, value.extract<uint64_t>());
      break;
    case FLOAT64:
      _offset += msgpack::pack_double(_output->data() + _offset, value.extract<double>());
      break;
    case FLOAT32:
      _offset += msgpack::pack_float(_output->data() + _offset, value.extract<float>());
      break;
    case BOOL:
      _offset += msgpack::pack_bool(_output->data() + _offset, value.extract<bool>());
      break;
    default:
      _offset += msgpack::pack_int(_output->data() + _offset, value.convert<int64_t>());
      break;
  }
  _count++;
}

void MsgpackMessageWriter::writeString(const FieldLeaf& leaf, const std::string& value) {
  writeKey(leaf);
  ensureCapacity(5 + value.size());
  _offset += msgpack::pack_string(_output->data() + _offset, value);
  _count++;
}

void MsgpackMessageWriter::writeEnum(const FieldLeaf& leaf, int32_t int_value,
                                    const std::string& /*enum_name*/) {
  writeKey(leaf);
  ensureCapacity(9);
  _offset += msgpack::pack_int(_output->data() + _offset, int_value);
  _count++;
}

void MsgpackMessageWriter::finish() {
  // Write map header at the start, then shift data if header is smaller than 5 bytes
  uint8_t header[5];
  uint32_t header_size = msgpack::pack_map(header, _count);

  if (header_size < 5) {
    // Shift data left to close the gap
    size_t gap = 5 - header_size;
    std::memmove(_output->data() + header_size, _output->data() + 5, _offset - 5);
    _offset -= gap;
  }
  std::memcpy(_output->data(), header, header_size);
  _output->resize(_offset);
}

}  // namespace RosMsgParser
