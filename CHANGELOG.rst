^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosx_introspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2026-02-21)
------------------
* Fix multiple bugs in serialization, deserialization, and JSON handling
  - Fix missing throw in ROS_Deserializer::deserialize for unsupported types
  - Fix NanoCDR_Deserializer::deserializeByteSequence crash on empty sequences
  - Fix ROS_Serializer buffer resize logic and UB (const-cast, type-punning)
  - Fix Tree::find using &_root instead of _root.get() on unique_ptr
  - Fix msgpack pointer invalidation after vector resize for large messages
  - Fix blob detection in deserialize() (was restricted to OTHER type only)
  - Fix deserializeIntoJson blob skip not advancing the deserializer
  - Fix serializeFromJson: add JSON parse error checking, type-safe value
  reading with range validation, proper null handling for missing fields
  - Add complete ROS_Serializer implementation (serialize, serializeString, etc.)
  - Add comprehensive test coverage for all fixes
  Co-Authored-By: Claude Opus 4.6 <noreply@anthropic.com>
* Merge pull request `#29 <https://github.com/facontidavide/rosx_introspection//issues/29>`_ from agxeed/fix/time-duration-nsec-serialization
  fix: correct nsecs duration serialization
* Merge pull request `#22 <https://github.com/facontidavide/rosx_introspection//issues/22>`_ from amarburg/bugfix_ament_export_testing_dependencies
  Add sensor_msgs, geometry_msgs as ament dependencies when BUILD_TESTING is true
* Merge pull request `#27 <https://github.com/facontidavide/rosx_introspection//issues/27>`_ from jorritolthuis/msgpack_cpp
  Make convertToMsgPack() available in C++
* Merge pull request `#30 <https://github.com/facontidavide/rosx_introspection//issues/30>`_ from agxeed/fix/compiler-warnings
  fix: compiler warnings
* Merge pull request `#32 <https://github.com/facontidavide/rosx_introspection//issues/32>`_ from konsim83/feature/replace_rosbag2_cpp_with_rclcpp
  Use rclcpp type support instead of rosbag2_cpp
* fix: compiler warnings
* fix: correct nsecs duration serialization
* Make convertToMsgPack() available in C++
* Add sensor_msgs, geometry_msgs as ament dependencies when BUILD_TESTING
* remove ament_target_dependencies
* Contributors: Davide Faconti, Jorrit Olthuis, Konrad Simon, Rein Appeldoorn, admin

2.0.1 (2025-10-01)
------------------
* Merge pull request `#21 <https://github.com/facontidavide/rosx_introspection/issues/21>`_ from bryzhao/fix/scikit-build-core-compatibility
  Fix CMakeLists to support scikit-build-core installations
* Fix CMakeLists to support scikit-build-core installations
* Merge pull request `#20 <https://github.com/facontidavide/rosx_introspection/issues/20>`_ from nealtanner/bugfix/mac-compliation
  fix compilation errors on mac
* replace sprintf with snprintf
* inline vs constexpr
* Contributors: Bryan Zhao, Davide Faconti, Neal Tanner

2.0.0 (2025-09-24)
------------------
* optionally remove JSON support
* new formatting
* fix memory and add README
* add python binding
* NanoCDR
* Merge pull request `#16 <https://github.com/facontidavide/rosx_introspection/issues/16>`_ from traversaro/patch-1
  Add missing include of functional header in message_definition_cache.cpp
* Fix issue `#13 <https://github.com/facontidavide/rosx_introspection/issues/13>`_
* Merge pull request `#11 <https://github.com/facontidavide/rosx_introspection/issues/11>`_ from valgur/bugfix/clang
  Fix ros_parser.cpp compilation failures on Clang
* Contributors: Davide Faconti, Martin Valgur, Silvio Traversaro

1.0.2 (2024-07-28)
------------------
* bug fix
* add unit tests
* Contributors: Davide Faconti

1.0.1 (2024-06-29)
------------------
* use CPM to download dependenies
* Merge pull request `#9 <https://github.com/facontidavide/rosx_introspection/issues/9>`_ from valgur/feature/cmake-improvements
* Contributors: Davide Faconti, Martin Valgur

1.0.0 (2024-06-26)
------------------
* New version including JSON conversion
* Contributors: Basavaraj-PN, Davide Faconti, ahmad-ra
