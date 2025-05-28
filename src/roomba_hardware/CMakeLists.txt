cmake_minimum_required(VERSION 3.16)
project(roomba_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(roomba_interfaces REQUIRED)

# Create the hardware interface library
add_library(
  roomba_hardware
  SHARED
  src/roomba_hardware_interface.cpp
)
target_compile_features(roomba_hardware PUBLIC cxx_std_17)
target_include_directories(roomba_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/roomba_hardware>
)
ament_target_dependencies(
  roomba_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  roomba_interfaces
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(roomba_hardware PRIVATE "ROOMBA_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface roomba_hardware_plugin.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/roomba_hardware
)
install(
  TARGETS roomba_hardware
  EXPORT export_roomba_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_targets(export_roomba_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle geometry_msgs roomba_interfaces)
ament_package()