cmake_minimum_required(VERSION 3.8)
project(cb_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(yarp_control_msgs REQUIRED)

add_library(cb_hw_interface
SHARED
src/cb_hw_interface.cpp)
target_compile_features(cb_hw_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(cb_hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  cb_hw_interface
  "rclcpp"
  "control_msgs"
  "lifecycle_msgs"
  "pluginlib"
  "rclcpp_lifecycle"
  "rcpputils"
  "rcutils"
  "hardware_interface"
  "yarp_control_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(cb_hw_interface PRIVATE "CB_HW_INTERFACE_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(cb_hw_interface cb_hw_interface.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS cb_hw_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  cb_hw_interface
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)

ament_package()
