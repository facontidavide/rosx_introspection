# CLAUDE.md - AI Assistant Guide for rosx_introspection

## Project Overview

**rosx_introspection** is a C++17 library for runtime parsing and introspection of ROS/ROS2 messages without compile-time type information. It is the unified successor to three earlier libraries: `ros_type_introspection`, `ros_msg_parser`, and `ros2_introspection`.

- **License:** MIT
- **Maintainer:** Davide Faconti
- **Namespace:** All code lives in the `RosMsgParser` namespace

## Repository Structure

```
rosx_introspection/
├── include/rosx_introspection/   # Public headers
│   ├── contrib/                  # Vendored third-party (SmallVector, span, msgpack, nanocdr)
│   ├── details/                  # Internal implementation details
│   ├── ros_utils/                # ROS2-specific utilities
│   ├── ros_parser.hpp            # Main parsing API, FlatMessage
│   ├── ros_type.hpp              # ROSType representation
│   ├── ros_message.hpp           # ROSMessage / ROSField structures
│   ├── ros_field.hpp             # Field metadata
│   ├── deserializer.hpp          # Deserialization interface (ROS1, ROS2, NanoCDR)
│   ├── serializer.hpp            # Serialization interface
│   ├── builtin_types.hpp         # Builtin type enums and utilities
│   ├── variant.hpp               # Type-safe variant for values
│   ├── tree.hpp                  # Generic tree data structure
│   ├── stringtree_leaf.hpp       # String tree leaf nodes
│   └── msgpack_utils.hpp         # MessagePack serialization
├── src/                          # Implementation files (.cpp)
├── python/                       # Python bindings (nanobind)
├── test/                         # Unit tests (GTest)
│   ├── test_parser.cpp           # Core parser tests (standalone)
│   ├── test_encoding.cpp         # Encoding/decoding tests (standalone)
│   ├── test_ros1.cpp             # ROS1-specific tests
│   └── test_ros2.cpp             # ROS2-specific tests
├── cmake/                        # Build helpers (CPM.cmake v0.40.0)
├── .github/workflows/ros2.yaml  # CI configuration
├── CMakeLists.txt                # Build configuration
├── package.xml                   # ROS2 package manifest (version 2.1.0)
├── conanfile.txt                 # Conan dependencies (fast-cdr)
└── .clang-format                 # Code formatting rules
```

## Build System

### CMake (version 3.10+)

The project supports three build modes:

1. **Standalone (no ROS):** Pure CMake build
2. **ROS1:** Uses catkin
3. **ROS2:** Uses ament_cmake / colcon

### Key CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `CMAKE_POSITION_INDEPENDENT_CODE` | ON | Enable -fPIC |
| `ROSX_HAS_JSON` | ON | JSON converters via RapidJSON |
| `ROSX_PYTHON_BINDINGS` | OFF | Python bindings via nanobind |
| `BUILD_TESTING` | OFF | Build unit tests |

### Building Standalone (without ROS)

```bash
mkdir build && cd build
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)
./parser_test       # Run standalone tests
```

### Building with ROS2

```bash
# In a colcon workspace
colcon build --packages-select rosx_introspection
colcon test --packages-select rosx_introspection
```

### Dependencies

- **RapidJSON** (v1.1.0) - Auto-downloaded via CPM if not found; conditional on `ROSX_HAS_JSON`
- **nanobind** (v2.9.2) - For Python bindings; conditional on `ROSX_PYTHON_BINDINGS`
- **fast-cdr** (v2.2.0) - Via Conan
- **GTest** - Required when `BUILD_TESTING=ON`
- **ROS2 packages** (when building with ament): `ament_index_cpp`, `rclcpp`, `rosbag2_cpp`
- **Test-only ROS2 packages:** `sensor_msgs`, `geometry_msgs`

## Testing

### Test Framework: Google Test

**Standalone tests** (no ROS required):
- `parser_test` — built from `test/test_parser.cpp` + `test/test_encoding.cpp`

**ROS2 tests** (require ROS2 environment):
- `rosx_introspection_test` — built from `test/test_parser.cpp` + `test/test_ros2.cpp`

### Running Tests

```bash
# Standalone
cd build && ./parser_test

# ROS2
colcon test --packages-select rosx_introspection --event-handlers console_direct+
```

## CI/CD

**GitHub Actions** (`.github/workflows/ros2.yaml`):
- Runs on push and pull requests
- Matrix: ROS2 Humble + ROS2 Jazzy on Ubuntu latest
- Uses `ros-industrial/industrial_ci@master`

## Code Conventions

### Formatting

- **Tool:** clang-format v17.0.6 (enforced via pre-commit hook)
- **Base style:** Google C++ with tweaks
- **Indent:** 2 spaces (no tabs)
- **Column limit:** 120
- **Braces:** Attach style, always inserted (`InsertBraces: true`)
- **Short blocks:** Empty blocks allowed on single line; short if-statements never on single line

Run formatter: `clang-format -i <file>`

### Naming Conventions

| Element | Convention | Example |
|---------|-----------|---------|
| Classes / Structs | PascalCase | `ROSMessage`, `Parser`, `ROSType` |
| Public methods | camelCase | `getSchema()`, `deserialize()` |
| Member variables | `_leading_underscore` + snake_case | `_schema`, `_global_warnings` |
| Enums | ALL_CAPS | `DISCARD_LARGE_ARRAYS`, `FLOAT64` |
| Type aliases | PascalCase | `MessageSchema`, `FieldTree` |
| Namespaces | PascalCase | `RosMsgParser` |

### Header Style

- Use `#pragma once` (no traditional include guards)
- All source files include MIT License header

### C++ Standards

- **Required:** C++17 (`cxx_std_17`)
- Smart pointers (`std::shared_ptr`, `std::unique_ptr`) for memory management
- `std::string_view` preferred for non-owning string references
- Templates used extensively for type-safe deserialization

### Pre-commit Hooks

Configured in `.pre-commit-config.yaml`:
- Standard checks: large files, YAML, merge conflicts, trailing whitespace, BOM
- C++ formatting: clang-format v17.0.6
- Excludes: `.vscode/`, `deps/`, `contrib/`

## Architecture

The core data flow:

```
Message Definition (string) → ROSMessage → Parser (registered schema)
Binary Data → Parser::deserialize() → FlatMessage (key/value pairs + blobs)
                    ↓
         deserializeIntoJson() → JSON output
         serializeFromJson()   → Binary serialization
```

**Key classes:**
- `ROSType` — Represents a ROS message type (package + name)
- `ROSField` — A single field within a message (name, type, array info)
- `ROSMessage` — Parsed message definition with ordered fields
- `Parser` — Main API: registers schemas, deserializes binary data into `FlatMessage`
- `FlatMessage` — Output container with named values, strings, and blobs
- `Variant` — Type-safe union for holding deserialized scalar values
- `Deserializer` — Abstract interface with ROS1, ROS2 (FastCDR), and NanoCDR implementations

## Key Files to Understand

When working on this codebase, start with:

1. `include/rosx_introspection/ros_parser.hpp` — The main public API
2. `src/ros_parser.cpp` — Core implementation (~580 lines)
3. `include/rosx_introspection/builtin_types.hpp` — Type system fundamentals
4. `include/rosx_introspection/deserializer.hpp` — Serialization layer
5. `test/test_parser.cpp` — Usage examples and expected behavior

## Things to Watch Out For

- **Multi-platform build:** Changes must work with standalone CMake, ROS1 (catkin), and ROS2 (ament). Guard ROS-specific code with `#ifdef USING_ROS2` or CMake conditionals.
- **Vendored code in `contrib/`:** Do not modify files in `include/rosx_introspection/contrib/` — these are third-party and excluded from pre-commit hooks.
- **Version consistency:** Version appears in both `CMakeLists.txt` (line 3) and `package.xml` (line 3). Note: these may currently differ — `package.xml` is the authoritative version (2.1.0).
- **Optional features:** JSON support (`ROSX_HAS_JSON`) and Python bindings (`ROSX_PYTHON_BINDINGS`) are compile-time options. Code guarded by `ROSX_HAS_JSON` preprocessor define.
