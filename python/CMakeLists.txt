cmake_minimum_required(VERSION 3.15)
project(rosx_introspection_python)

# Find Python
find_package(Python 3.8
  REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)

# Fetch nanobind
include(FetchContent)
FetchContent_Declare(
  nanobind
  GIT_REPOSITORY https://github.com/wjakob/nanobind
  GIT_TAG        v2.2.0  # Use the latest stable version
)
FetchContent_MakeAvailable(nanobind)

# Create the Python module
nanobind_add_module(
  rosx_introspection_py
  STABLE_ABI
  NB_STATIC  # Build static for better portability
  rosx_introspection_py.cpp
)

# Link with the main rosx_introspection library
target_link_libraries(rosx_introspection_py PRIVATE rosx_introspection)

# Set properties
set_target_properties(rosx_introspection_py PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  OUTPUT_NAME "rosx_introspection"  # This makes the .so file have the right name
)

# Install the Python module
install(TARGETS rosx_introspection_py
  LIBRARY DESTINATION ${Python_SITEARCH}/rosx_introspection)

# Create __init__.py
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/__init__.py"
"\"\"\"ROS X Introspection - Python bindings for ROS message parsing.\"\"\"

from .rosx_introspection import Parser

__version__ = '1.0.0'
__all__ = ['Parser', 'parse_ros_message']
")

# Install __init__.py
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/__init__.py"
  DESTINATION ${Python_SITEARCH}/rosx_introspection)