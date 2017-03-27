cmake_minimum_required(VERSION 3.5)

project(pg_mixed)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(python_cmake_module REQUIRED)
find_package(PythonExtra MODULE)

# configure_file("pg_mixed/setup.py" "pg_mixed/__init__.py" @ONLY)
ament_python_install_package(${PROJECT_NAME})

add_executable(${PROJECT_NAME}_talker src/talker.cpp)
ament_target_dependencies(${PROJECT_NAME}_talker rclcpp std_msgs)

install(TARGETS
  ${PROJECT_NAME}_talker
  DESTINATION bin
)

#add_library(${PROJECT_NAME}_listener listener.py)
#include_directories(${PYTHON_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME}_listener ${PYTHON_LIBRARIES})
#ament_target_dependencies(${PROJECT_NAME}_listener "rcl"  ${PythonExtra_LIBRARIES})

#add_subdirectory(pg_mixed)

ament_package()
