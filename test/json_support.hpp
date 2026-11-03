#pragma once

#include <stdexcept>
#include <string>

#include "rosx_introspection/deserializer.hpp"
#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/serializer.hpp"

namespace RosMsgParser::test {

/// True if the library was built with ROSX_HAS_JSON. The definition is private to the
/// library, so the tests ask the parser instead of checking the macro.
inline bool HasJsonSupport() {
  Parser parser("topic", ROSType("my_pkg/Test"), "uint32 value\n");
  NanoCDR_Serializer serializer;
  serializer.reset();
  serializer.serialize(UINT32, Variant(uint32_t(0)));

  NanoCDR_Deserializer deserializer;
  std::string json;
  try {
    return parser.deserializeIntoJson(
        Span<const uint8_t>(reinterpret_cast<const uint8_t*>(serializer.getBufferData()), serializer.getBufferSize()),
        &json, &deserializer);
  } catch (const std::runtime_error& ex) {
    if (std::string(ex.what()).find("without JSON support") != std::string::npos) {
      return false;
    }
    throw;
  }
}

}  // namespace RosMsgParser::test

/// Skips the current test when the library was built without JSON support.
#define SKIP_WITHOUT_JSON()                                  \
  do {                                                       \
    if (!RosMsgParser::test::HasJsonSupport()) {             \
      GTEST_SKIP() << "JSON support disabled in this build"; \
    }                                                        \
  } while (0)
