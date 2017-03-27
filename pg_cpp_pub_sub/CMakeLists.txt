cmake_minimum_required(VERSION 3.5)

project(pg_cpp_pub_sub)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(steelcast_msgs REQUIRED)

add_executable(pg_cpp_pub src/pg_cpp_pub.cpp)
ament_target_dependencies(pg_cpp_pub rclcpp std_msgs sensor_msgs steelcast_msgs)

add_executable(pg_cpp_sub src/pg_cpp_sub.cpp)
ament_target_dependencies(pg_cpp_sub rclcpp std_msgs sensor_msgs steelcast_msgs)

install(TARGETS
  pg_cpp_pub
  pg_cpp_sub
  DESTINATION bin
)

ament_package()
