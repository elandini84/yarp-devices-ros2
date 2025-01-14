cmake_minimum_required(VERSION 3.16)
project(yarp-devices-ros2
        LANGUAGES CXX
        VERSION 3.7.100)

include(FeatureSummary)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(YCM 0.12 REQUIRED)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "yarp-devices-ros2 is always built with dynamic plugins")
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

option(YARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs "If ON, use map2d_nws_ros2_msgs found in the system, otherwise build it with this project." OFF)
option(YARP_ROS2_USE_SYSTEM_yarp_control_msgs "If ON, use yarp_control_msgs found in the system, otherwise build it with this project." OFF)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS 1)

include(AddUninstallTarget)

find_package(YARP 3.7.2 COMPONENTS os sig dev REQUIRED)
set(CMAKE_C_FLAGS "${YARP_C_FLAGS} ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${YARP_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

yarp_configure_plugins_installation(yarp-devices-ros2)

find_package(ament_cmake REQUIRED)
find_package(Doxygen)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(test_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)

if(YARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs)
  find_package(map2d_nws_ros2_msgs REQUIRED)
else()
  add_subdirectory(ros2_interfaces_ws/src/map2d_nws_ros2_msgs)
endif()

if(YARP_ROS2_USE_SYSTEM_yarp_control_msgs)
  find_package(yarp_control_msgs REQUIRED)
else()
  add_subdirectory(ros2_interfaces_ws/src/yarp_control_msgs)
endif()

add_subdirectory(src)
#add_subdirectory(doc)

#if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  ## the following line skips the linter which checks for copyrights
  ## uncomment the line when a copyright and license is not present in all source files
  ##set(ament_cmake_copyright_FOUND TRUE)
  ## the following line skips cpplint (only works in a git repo)
  ## uncomment the line when this package is not in a git repo
  ##set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
#endif()

ament_package()

feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)
