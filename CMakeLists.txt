cmake_minimum_required(VERSION 3.10)

project(rosx_introspection  LANGUAGES C CXX VERSION 2.0.0)

# Build flags
#---- Enable C++17 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
    set(USING_ROS2 TRUE)
    find_package(rosbag2_cpp REQUIRED)
    find_package(ament_index_cpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(fastcdr REQUIRED)

    set(EXTRA_SRC
       src/ros_utils/message_definition_cache.cpp
       src/ros_utils/ros2_helpers.cpp)
endif()

find_package(RapidJSON REQUIRED)

if(NOT USING_ROS2 )
    message(STATUS "[FastCdr] not found, create static libraries")
    # Override Fast-CDR option: compile as static lib
    SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Create static libraries by default")
    add_subdirectory(3rdparty/Fast-CDR)
    include_directories(3rdparty/Fast-CDR/include)
else()
    # Mention that FastCdr has been found on system
    message(STATUS "[FastCdr] found, version: ${fastcdr_VERSION}")
endif()

###############################################
## Declare a C++ library
###############################################
add_library(rosx_introspection STATIC
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

target_link_libraries(rosx_introspection )

target_include_directories(rosx_introspection PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

if(USING_ROS2)
    ament_target_dependencies(rosx_introspection
        ament_index_cpp
        rclcpp
        rosbag2_cpp
        fastcdr)
endif()

install(TARGETS rosx_introspection
    EXPORT rosx_introspection
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

###############################################
## Tests
###############################################

if(USING_ROS2)
    add_executable(rosx_introspection_test
        test/test_parser.cpp
        test/test_ros2.cpp )

    ament_target_dependencies(rosx_introspection_test sensor_msgs)

    target_link_libraries(rosx_introspection_test rosx_introspection)

    target_include_directories(rosx_introspection_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/doctest>)
endif()
