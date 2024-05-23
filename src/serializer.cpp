#include "rosx_introspection/serializer.hpp"
#include "fastcdr/Cdr.h"

/* make sure to remain compatible with previous version of fastCdr */
#if ((FASTCDR_VERSION_MAJOR < 2))
#define get_buffer_pointer() getBufferPointer()
#define get_current_position() getCurrentPosition()
#define get_serialized_data_length() getSerializedDataLength()
#define CdrVersion Cdr
#endif

namespace RosMsgParser
{

FastCDR_Serializer::FastCDR_Serializer()
{
  using namespace eprosima::fastcdr;
  _cdr_buffer = std::make_shared<FastBuffer>();
  _cdr = std::make_shared<Cdr>(*_cdr_buffer, Cdr::DEFAULT_ENDIAN, CdrVersion::DDS_CDR);
}

void FastCDR_Serializer::serialize(BuiltinType type, const Variant& val)
{
  switch (type)
  {
    case BuiltinType::CHAR:
    case BuiltinType::UINT8:
      _cdr->serialize(val.convert<uint8_t>());
      break;

    case BuiltinType::BOOL:
    case BuiltinType::BYTE:
    case BuiltinType::INT8:
      _cdr->serialize(val.convert<int8_t>());

    case BuiltinType::UINT16:
      _cdr->serialize(val.convert<uint16_t>());
      break;
    case BuiltinType::INT16:
      _cdr->serialize(val.convert<int16_t>());
      break;
    case BuiltinType::UINT32:
      _cdr->serialize(val.convert<uint32_t>());
      break;
    case BuiltinType::INT32:
      _cdr->serialize(val.convert<int32_t>());
      break;
    case BuiltinType::UINT64:
      _cdr->serialize(val.convert<uint64_t>());
      break;
    case BuiltinType::INT64:
      _cdr->serialize(val.convert<int64_t>());
      break;
    case BuiltinType::FLOAT32:
      _cdr->serialize(val.convert<float>());
      break;
    case BuiltinType::FLOAT64:
      _cdr->serialize(val.convert<double>());
      break;
    default:
      throw std::runtime_error("Unsupported type");
  }
}

void FastCDR_Serializer::serializeString(const std::string& str)
{
  _cdr->serialize(str);
}

void FastCDR_Serializer::serializeUInt32(uint32_t value)
{
  _cdr->serialize(value);
}

void FastCDR_Serializer::reset()
{
  _cdr->reset();
}

const char* FastCDR_Serializer::getBufferData() const
{
  return _cdr->get_buffer_pointer();
}

size_t FastCDR_Serializer::getBufferSize() const
{
  return _cdr->get_serialized_data_length();
}

void FastCDR_Serializer::writeHeader()
{
  _cdr->reset();
  _cdr->serialize_encapsulation();
}

}  // namespace RosMsgParser
