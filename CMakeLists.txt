################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(hokuyo3d)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system)

################################################################################
# Build
################################################################################
include_directories(
  include 
  ${Boost_INCLUDE_DIRS}
)

add_library(raspimouse_component SHARED src/raspimouse_component.cpp)
target_compile_definitions(raspimouse_component PRIVATE "RASPIMOUSE_BUILDING_DLL")
ament_target_dependencies(raspimouse_component
  rclcpp
  rclcpp_components
  sensor_msgs
  )
rclcpp_components_register_nodes(raspimouse_component "raspimouse::Raspimouse")

add_executable(hokuyo3d src/hokuyo3d.cpp)
target_link_libraries(raspimouse raspimouse_component)
ament_target_dependencies(raspimouse
  rclcpp
  class_loader
  sensor_msgs)

ament_export_targets(export_raspimouse_component HAS_LIBRARY_TARGET)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(raspimouse_msgs)

ament_export_include_directories(include)
ament_export_libraries(raspimouse_component)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS raspimouse_component
  EXPORT export_raspimouse_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS
  raspimouse
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
