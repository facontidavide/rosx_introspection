#pragma once

// API adapted to FastCDR

#include <exception>
#include <vector>
#include "rosx_introspection/builtin_types.hpp"
#include "rosx_introspection/variant.hpp"

namespace eprosima::fastcdr
{
class FastBuffer;
class Cdr;
}  // namespace eprosima::fastcdr

namespace RosMsgParser
{

class Serializer
{
public:
  virtual ~Serializer() = default;

  virtual bool isROS2() const = 0;

  virtual void serialize(BuiltinType type, const Variant& val) = 0;

  virtual void serializeString(const std::string& str) = 0;

  virtual void serializeUInt32(uint32_t value) = 0;

  virtual void reset() = 0;

  virtual const char* getBufferData() const = 0;

  virtual size_t getBufferSize() const = 0;

  virtual void writeHeader() = 0;
};

//-----------------------------------------------------------------

// Specialization of serializer that works with ROS1
class ROS_Serializer : public Serializer
{
public:
  bool isROS2() const override
  {
    return false;
  }

  void serialize(BuiltinType type, const Variant& val) override;

  void serializeString(const std::string& str) override;

  void serializeUInt32(uint32_t value) override;

  void reset() override;

  const char* getBufferData() const override;

  size_t getBufferSize() const override;

protected:
  std::vector<uint8_t> _buffer;
  size_t _current_size = 0;

  template <typename T>
  T serialize(const T& val)
  {
    T out;
    if (_current_size + sizeof(T) > _buffer.size())
    {
      _buffer.resize((_current_size * 3) / 2);
    }
    auto* ptr = &(_buffer[_current_size]);
    *(reinterpret_cast<const T*>(ptr)) = val;
    _current_size += sizeof(T);
    return out;
  }
};

//-----------------------------------------------------------------

// Specialization od deserializer that works with ROS2
// wrapping FastCDR
class FastCDR_Serializer : public Serializer
{
public:
  FastCDR_Serializer();

  bool isROS2() const override
  {
    return true;
  }

  void serialize(BuiltinType type, const Variant& val) override;

  void serializeString(const std::string& str) override;

  void serializeUInt32(uint32_t value) override;

  void reset() override;

  const char* getBufferData() const override;

  size_t getBufferSize() const override;

  void writeHeader() override;

protected:
  std::shared_ptr<eprosima::fastcdr::FastBuffer> _cdr_buffer;
  std::shared_ptr<eprosima::fastcdr::Cdr> _cdr;
};

using ROS2_Serializer = FastCDR_Serializer;

}  // namespace RosMsgParser
