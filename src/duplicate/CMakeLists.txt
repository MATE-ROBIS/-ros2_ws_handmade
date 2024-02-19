cmake_minimum_required(VERSION 3.5)
project(duplicate)



# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(PkgConfig)
pkg_check_modules(modbus libmodbus)
include_directories(include mdrtu.hpp)
add_executable(mod_node src/mdrtusrc.cpp)


target_link_libraries(mod_node modbus)

add_executable(duplicate_node src/simple_publisher.cpp)
add_executable(parameter_node src/parameter.cpp)
ament_target_dependencies(duplicate_node rclcpp)
ament_target_dependencies(parameter_node rclcpp)

install(TARGETS
  duplicate_node
  parameter_node

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
