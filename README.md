# ROS X Introspection

[![Ubuntu](https://github.com/facontidavide/rosx_introspection/actions/workflows/ubuntu.yaml/badge.svg)](https://github.com/facontidavide/rosx_introspection/actions/workflows/ubuntu.yaml)
[![Humble](https://github.com/facontidavide/rosx_introspection/actions/workflows/humble.yaml/badge.svg)](https://github.com/facontidavide/rosx_introspection/actions/workflows/humble.yaml)
[![Jazzy](https://github.com/facontidavide/rosx_introspection/actions/workflows/jazzy.yaml/badge.svg)](https://github.com/facontidavide/rosx_introspection/actions/workflows/jazzy.yaml)
[![Kilted](https://github.com/facontidavide/rosx_introspection/actions/workflows/kilted.yaml/badge.svg)](https://github.com/facontidavide/rosx_introspection/actions/workflows/kilted.yaml)
[![Rolling](https://github.com/facontidavide/rosx_introspection/actions/workflows/rolling.yaml/badge.svg)](https://github.com/facontidavide/rosx_introspection/actions/workflows/rolling.yaml)

A runtime message parser and introspection library for ROS. It can deserialize any ROS message
into key/value pairs without compile-time type knowledge.

## Supported schema formats

- **ROS .msg** (ROS1 and ROS2 message definitions)
- **DDS IDL** (OMG IDL 4.2 subset) -- including enums, unions, `@key`, `@optional`, multi-dimensional arrays, and struct inheritance.

## Build modes

The library compiles in two modes:

- **ROS2** (colcon/ament)
- **Standalone** (vanilla CMake, no ROS dependency)

ROS1 `.msg` schemas and the ROS1 serialization format (`ROS_Deserializer`) are still supported at runtime,
but the catkin build system is no longer maintained.

## How it works

To parse a message at runtime, you need:

1. The **type name** (e.g. `sensor_msgs/JointState` or `my_pkg::MyIdlMessage`)
2. The **schema definition** (a `.msg` file or IDL text)
3. A **raw memory buffer** containing the serialized message

The raw buffer is typically obtained from:

- [rosbag::MessageInstance](https://docs.ros.org/en/noetic/api/rosbag_storage/html/c++/classrosbag_1_1MessageInstance.html)
  or [topic_tools::ShapeShifter](http://docs.ros.org/en/noetic/api/topic_tools/html/shape__shifter_8h.html) in **ROS1**.
- [GenericSubscription](https://github.com/ros2/rclcpp/blob/rolling/rclcpp/src/rclcpp/generic_subscription.cpp)
  or `rosbag2_storage::SerializedBagMessage` in **ROS2**.
- [MCAP](https://github.com/foxglove/mcap) files (works without ROS).

## Output writers

The `MessageWriter` interface allows different output formats from the same deserialization walk:

| Writer | Description |
|--------|-------------|
| `FlatMessageWriter` | Produces a `FlatMessage` (vector of key/value pairs). Default output. |
| `MsgpackMessageWriter` | Writes MessagePack binary directly, bypassing `FlatMessage`. |
| `JsonMessageWriter` | Produces a JSON document (requires `ROSX_HAS_JSON=ON`). |

Custom writers can be implemented by subclassing `MessageWriter`.

## Building and testing

```bash
# Standalone build (no ROS)
cmake -S. -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build
```

### Benchmarks

The MCAP benchmark measures deserialization throughput on real bag files:

```bash
cmake -S. -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_BENCHMARKS=ON
cmake --build build

# Run with different output writers
./build/mcap_benchmark path/to/file.mcap --iterations 5 --writer flat
./build/mcap_benchmark path/to/file.mcap --iterations 5 --writer msgpack
./build/mcap_benchmark path/to/file.mcap --iterations 5 --writer json
```

The IDL benchmark measures CDR deserialization performance:

```bash
./build/idl_benchmark
```

## Python binding

```bash
cmake -S. -B build_python -DCMAKE_BUILD_TYPE=Release -DROSX_PYTHON_BINDINGS=ON
cmake --build build_python

PYTHONPATH=build_python/python python3 python/mcap_ros_parser.py path_to_your_rosbag.mcap
```
