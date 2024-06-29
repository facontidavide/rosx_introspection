cmake_minimum_required(VERSION 3.10)

project(rosx_introspection LANGUAGES CXX VERSION 1.0.0)


option(CMAKE_POSITION_INDEPENDENT_CODE "Set -fPIC" ON)

include(cmake/CPM.cmake)

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
    set(USING_ROS2 TRUE)
    find_package(rosbag2_cpp REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(fastcdr REQUIRED)

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
        rosbag2_cpp
        fastcdr)
else()
    message(STATUS "Downloading FastCDR with CPM and building as static library")
    # Override Fast-CDR option: compile as static lib
    # set(BUILD_SHARED_LIBS OFF CACHE BOOL "Create static libraries by default")

    CPMAddPackage(NAME fastcdr
                  GIT_TAG v2.2.2
                  GITHUB_REPOSITORY "eProsima/Fast-CDR"
                  OPTIONS "BUILD_SHARED_LIBS OFF")
    target_link_libraries(rosx_introspection fastcdr)
endif()


find_package(RapidJSON QUIET)

if(NOT RapidJSON_FOUND)
    message(STATUS "Downloading RapidJSON with CPM")    
    CPMAddPackage(NAME rapidjson
        GIT_TAG v1.1.0
        GITHUB_REPOSITORY "Tencent/rapidjson"
        OPTIONS "RAPIDJSON_BUILD_EXAMPLES OFF" "RAPIDJSON_BUILD_TESTS OFF" "RAPIDJSON_BUILD_DOC OFF")
endif()

target_link_libraries(rosx_introspection rapidjson)


###############################################
## Install and Tests
###############################################

if(USING_ROS2)

    if(BUILD_TESTING)

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
    endif()

    ament_export_targets(rosx_introspectionTargets HAS_LIBRARY_TARGET)
    ament_export_dependencies(ament_index_cpp rosbag2_cpp fastcdr)
    ament_package()

endif()

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
