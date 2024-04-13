cmake_minimum_required(VERSION 3.10)

project(rosx_introspection  LANGUAGES C CXX VERSION 2.0.0)

# Build flags
#---- Enable C++17 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT fastcdr_FOUND )
    message(STATUS "[FastCdr] not found, create shared libraries")
    # Override Fast-CDR option: compile as static lib
    SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Create shared libraries by default")
    add_subdirectory(3rdparty/Fast-CDR)
    include_directories(3rdparty/Fast-CDR/include)
else()
    # Mention that FastCdr has been found on system
    message(STATUS "[FastCdr] found, version: ${fastcdr_VERSION}")
endif()

if("$ENV{ROS_VERSION}" STREQUAL "2")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# Add include directories
include_directories(
    include
    3rdparty
    ${rclcpp_INCLUDE_DIRS}
    ${rosbag2_cpp_INCLUDE_DIRS}
    ${rosidl_typesupport_INCLUDE_DIRS}
)

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
    src/ros_utils/ros2_helpers.cpp
    src/ros_utils/message_definition_cache.cpp
    3rdparty/Fast-CDR/src/cpp/Cdr.cpp
    )

ament_target_dependencies(rosx_introspection
    rclcpp
    rosbag2_cpp
    rosidl_typesupport_cpp
)


ament_export_targets(export_rosx_introspection HAS_LIBRARY_TARGET)

# Install libraries
install(
    TARGETS rosx_introspection
    EXPORT export_rosx_introspection
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include

)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY 3rdparty/rapidjson/
        DESTINATION include/rapidjson)

ament_package()


else()

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

include_directories(
    include
    3rdparty
    ${catkin_INCLUDE_DIRS}
)
###############################################
## Declare a C++ library
###############################################
catkin_package(
    INCLUDE_DIRS include 3rdparty
    LIBRARIES rosx_introspection
)

add_library(rosx_introspection STATIC
    ${SRC_FILES}
    src/ros_type.cpp
    src/ros_field.cpp
    src/stringtree_leaf.cpp
    src/ros_message.cpp
    src/ros_parser.cpp
    src/deserializer.cpp
    )

target_link_libraries(rosx_introspection
    fastcdr
    ${catkin_LIBRARIES} )



endif()