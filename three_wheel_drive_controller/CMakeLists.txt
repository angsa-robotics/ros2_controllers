cmake_minimum_required(VERSION 3.16)
project(three_wheel_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcppmath QUIET)
find_package(rcpputils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

generate_parameter_library(three_wheel_drive_controller_parameters
  src/three_wheel_drive_controller_parameter.yaml
)

add_library(three_wheel_drive_controller SHARED
  src/three_wheel_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_compile_features(three_wheel_drive_controller PUBLIC cxx_std_17)
target_include_directories(three_wheel_drive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/three_wheel_drive_controller>
)
target_link_libraries(three_wheel_drive_controller PUBLIC three_wheel_drive_controller_parameters)
ament_target_dependencies(three_wheel_drive_controller PUBLIC
  controller_interface
  control_toolbox
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)
# add rcppmath if found, otherwise it will use rcpputils (for Humble support)
if(rcppmath_FOUND)
  target_link_libraries(three_wheel_drive_controller PUBLIC rcppmath::rcppmath)
endif()

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(three_wheel_drive_controller PRIVATE "THREE_WHEEL_DRIVE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface three_wheel_drive_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/three_wheel_drive_controller
)
install(TARGETS three_wheel_drive_controller three_wheel_drive_controller_parameters
  EXPORT export_three_wheel_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_three_wheel_drive_controller
    test/test_load_three_wheel_drive_controller.cpp
  )
  target_link_libraries(test_load_three_wheel_drive_controller three_wheel_drive_controller)
  ament_target_dependencies(test_load_three_wheel_drive_controller
    controller_manager
    ros2_control_test_assets
  )

  # ament_add_gmock(test_three_wheel_drive_controller
  #   test/test_three_wheel_drive_controller.cpp
  # )
  # target_link_libraries(test_three_wheel_drive_controller three_wheel_drive_controller)
  # ament_target_dependencies(test_three_wheel_drive_controller
  #   controller_manager
  #   geometry_msgs
  #   hardware_interface
  #   nav_msgs
  #   rclcpp
  #   rclcpp_lifecycle
  #   realtime_tools
  #   tf2
  #   tf2_msgs
  # )
endif()

ament_export_targets(export_three_wheel_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()