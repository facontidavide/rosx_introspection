cmake_minimum_required(VERSION 3.10)

project(rosx_introspection LANGUAGES CXX VERSION 2.1.0)

# create compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(CMAKE_POSITION_INDEPENDENT_CODE "Set -fPIC" ON)
option(ROSX_PYTHON_BINDINGS "Build Python bindings using nanobind" OFF)
option(ROSX_HAS_JSON "Add JSON converters" ON)

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
    src/ros_type.cpp
    src/ros_field.cpp
    src/stringtree_leaf.cpp
    src/ros_message.cpp
    src/ros_parser.cpp
    src/deserializer.cpp
    src/serializer.cpp
    src/msgpack_utils.cpp
    ${EXTRA_SRC}
    )

target_include_directories(rosx_introspection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(rosx_introspection PUBLIC cxx_std_17)

if(USING_ROS2)
    target_link_libraries(rosx_introspection PUBLIC
        ament_index_cpp::ament_index_cpp
        rclcpp::rclcpp
        rosbag2_cpp::rosbag2_cpp
    )
endif()

if(ROSX_HAS_JSON)
    find_package(RapidJSON QUIET)

    # Verify the found headers actually exist
    if(RapidJSON_FOUND AND RapidJSON_INCLUDE_DIRS)
        if(NOT EXISTS "${RapidJSON_INCLUDE_DIRS}/rapidjson/document.h")
            set(RapidJSON_FOUND FALSE)
        endif()
    endif()

    if(NOT RapidJSON_FOUND)
        message(STATUS "Downloading RapidJSON with CPM")
        CPMAddPackage(NAME rapidjson
            GIT_TAG v1.1.0
            GITHUB_REPOSITORY "Tencent/rapidjson"
            OPTIONS "RAPIDJSON_BUILD_EXAMPLES OFF" "RAPIDJSON_BUILD_TESTS OFF" "RAPIDJSON_BUILD_DOC OFF")
    endif()

    target_compile_definitions(rosx_introspection PRIVATE ROSX_HAS_JSON)
    if(NOT USING_ROS2)
        if(RapidJSON_INCLUDE_DIRS)
            target_include_directories(rosx_introspection PRIVATE ${RapidJSON_INCLUDE_DIRS})
        elseif(rapidjson_SOURCE_DIR)
            target_include_directories(rosx_introspection PRIVATE ${rapidjson_SOURCE_DIR}/include)
        elseif(TARGET rapidjson)
            target_link_libraries(rosx_introspection PRIVATE rapidjson)
        endif()
    endif()

endif(ROSX_HAS_JSON)


###############################################
## Install and Tests
###############################################
if(BUILD_TESTING)
    if(USING_ROS2)
        find_package(sensor_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(ament_cmake_gtest REQUIRED)

        set(TESTING_DEPENDENCIES sensor_msgs geometry_msgs)

        ament_add_gtest(rosx_introspection_test
            test/test_parser.cpp
            test/test_ros2.cpp )

        target_link_libraries(rosx_introspection PUBLIC
            ${sensor_msgs_TARGETS}
            ${geometry_msgs_TARGETS}
            )
        target_link_libraries(rosx_introspection_test rosx_introspection)

        target_include_directories(rosx_introspection_test PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    endif(USING_ROS2)

    find_package(GTest QUIET)
    if(NOT GTest_FOUND)
        message(STATUS "Downloading GTest with CPM")
        CPMAddPackage(NAME googletest
            GIT_TAG v1.14.0
            GITHUB_REPOSITORY "google/googletest"
            OPTIONS "BUILD_GMOCK OFF" "INSTALL_GTEST OFF")
    endif()

    add_executable(parser_test
            test/test_parser.cpp
            test/test_encoding.cpp)

    if(TARGET GTest::gtest)
        target_link_libraries(parser_test rosx_introspection GTest::gtest GTest::gtest_main)
    else()
        target_link_libraries(parser_test rosx_introspection GTest::GTest GTest::Main)
    endif()
    target_include_directories(parser_test PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

    include(GoogleTest)
    gtest_discover_tests(parser_test)

endif(BUILD_TESTING)


if(USING_ROS2)
    ament_export_targets(rosx_introspectionTargets HAS_LIBRARY_TARGET)
    ament_export_dependencies(ament_index_cpp rosbag2_cpp ${TESTING_DEPENDENCIES})
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

###############################################
## Benchmarks
###############################################
option(BUILD_BENCHMARKS "Build benchmark executables" OFF)
if(BUILD_BENCHMARKS AND USING_ROS2)
    add_executable(mcap_benchmark test/benchmark_mcap.cpp)
    target_link_libraries(mcap_benchmark rosx_introspection)
endif()

# Python bindings
if(ROSX_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()
