cmake_minimum_required(VERSION 3.10)

project(rosx_introspection LANGUAGES CXX VERSION 1.0.2)

# create compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(CMAKE_POSITION_INDEPENDENT_CODE "Set -fPIC" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings using nanobind" OFF)

include(cmake/CPM.cmake)

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
    set(USING_ROS2 TRUE)
    find_package(rosbag2_cpp REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(rclcpp REQUIRED)

    set(EXTRA_SRC
       src/ros_utils/message_definition_cache.cpp
       src/ros_utils/ros2_helpers.cpp)
endif()


###############################################
## Declare a C++ library
###############################################
add_library(rosx_introspection
    ${SRC_FILES}
    src/ros_type.cpp
    src/ros_field.cpp
    src/stringtree_leaf.cpp
    src/ros_message.cpp
    src/ros_parser.cpp
    src/deserializer.cpp
    src/serializer.cpp
    ${EXTRA_SRC}
    )

target_include_directories(rosx_introspection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(rosx_introspection PUBLIC cxx_std_17)

if(USING_ROS2)
    ament_target_dependencies(rosx_introspection
        ament_index_cpp
        rclcpp
        rosbag2_cpp)
endif()


find_package(RapidJSON QUIET)

if(NOT RapidJSON_FOUND)
    message(STATUS "Downloading RapidJSON with CPM")
    CPMAddPackage(NAME rapidjson
        GIT_TAG v1.1.0
        GITHUB_REPOSITORY "Tencent/rapidjson"
        OPTIONS "RAPIDJSON_BUILD_EXAMPLES OFF" "RAPIDJSON_BUILD_TESTS OFF" "RAPIDJSON_BUILD_DOC OFF")
endif()


###############################################
## Install and Tests
###############################################
if(BUILD_TESTING)
    if(USING_ROS2)
        find_package(sensor_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(ament_cmake_gtest REQUIRED)

        ament_add_gtest(rosx_introspection_test
            test/test_parser.cpp
            test/test_ros2.cpp )

        ament_target_dependencies(rosx_introspection_test sensor_msgs geometry_msgs)
        target_link_libraries(rosx_introspection_test rosx_introspection)

        target_include_directories(rosx_introspection_test PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    endif(USING_ROS2)

    find_package(GTest REQUIRED)

    add_executable(parser_test
            test/test_parser.cpp
            test/test_encoding.cpp)

    target_link_libraries(parser_test rosx_introspection GTest::GTest GTest::Main)
    target_include_directories(parser_test PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

endif(BUILD_TESTING)


if(USING_ROS2)
    ament_export_targets(rosx_introspectionTargets HAS_LIBRARY_TARGET)
    ament_export_dependencies(ament_index_cpp rosbag2_cpp)
    ament_package()
endif(USING_ROS2)

include(GNUInstallDirs)
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(TARGETS rosx_introspection
    EXPORT rosx_introspectionTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Python bindings
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()
