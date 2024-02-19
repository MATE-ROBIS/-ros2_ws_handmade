cmake_minimum_required(VERSION 3.5)
project(rtp_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()


find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig)

pkg_check_modules(modbus REQUIRED libmodbus)






add_library(rtp_control SHARED src/diff_hardware.cpp src/wheel.cpp src/rtp_modbus.cpp )




target_include_directories(
  rtp_control
  PRIVATE
  include
)



ament_target_dependencies(
  rtp_control
  hardware_interface
  controller_manager
  modbus
  rclcpp
  pluginlib
  control_toolbox
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)









install(
  TARGETS rtp_control
  DESTINATION lib
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



ament_export_libraries(
  rtp_control
  pluginlib
  hardware_interface
  
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
  control_toolbox
)

ament_package()