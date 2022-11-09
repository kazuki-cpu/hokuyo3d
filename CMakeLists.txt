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

add_library(hokuyo3d_driver SHARED src/hokuyo3d_driver.cpp)
target_compile_definitions(hokuyo3d_driver PRIVATE "HOKUYO3D_BUILDING_DLL")
ament_target_dependencies(hokuyo3d_driver
  rclcpp
  rclcpp_components
  sensor_msgs
  )
rclcpp_components_register_nodes(hokuyo3d_driver "hokuyo3d::Hokuyo3dNode")

add_executable(hokuyo3d src/hokuyo3d.cpp)
target_link_libraries(hokuyo3d hokuyo3d_driver)
ament_target_dependencies(hokuyo3d
  rclcpp
  class_loader
  sensor_msgs)

ament_export_targets(export_hokuyo3d_driver HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(sensor_msgs)


ament_export_include_directories(include)
ament_export_libraries(hokuyo3d_driver)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS hokuyo3d_driver
  EXPORT export_hokuyo3d_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(
  TARGETS hokuyo3d
  DESTINATION lib/${PROJECT_NAME})

ament_package()
