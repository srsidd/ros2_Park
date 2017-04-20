cmake_minimum_required(VERSION 3.5)

project(pg_cpp_pub_sub)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

# Find Python3 packages
find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs 3 REQUIRED)

find_path(PYTHON_INCLUDE_PATH Python.h
  /usr/include
  /usr/local/include)

include_directories(${PYTHON_INCLUDE_DIRECTORIES})

set(BOOST_ROOT /usr/local/include/boost)
set(BOOST_LIBRARYDIR /usr/local/lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost COMPONENTS system python3 REQUIRED)

add_executable(pg_cpp_pub src/pg_cpp_pub.cpp)
ament_target_dependencies(pg_cpp_pub rclcpp std_msgs sensor_msgs steelcast_msgs)

add_executable(pg_cpp_sub src/pg_cpp_sub.cpp)
ament_target_dependencies(pg_cpp_sub rclcpp std_msgs sensor_msgs steelcast_msgs)

add_library(hi SHARED src/pg_export_py.cpp)
ament_target_dependencies(hi rclcpp std_msgs)
set_target_properties(hi PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ~/ros2_overlay_ws/install/lib/python3.5/site-packages)
target_link_libraries(hi ${PYTHON_LIBRARIES} boost_system boost_python3)

install(TARGETS
  pg_cpp_pub
  pg_cpp_sub
  DESTINATION bin
)

ament_package()
