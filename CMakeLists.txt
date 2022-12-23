cmake_minimum_required(VERSION 3.5)
project(hokuyo3d)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Boost REQUIRED system)


include_directories(
  include 
  ${Boost_INCLUDE_DIRS}
)

add_library(hokuyo3d_driver SHARED src/hokuyo3d_driver.cpp)
target_compile_definitions(hokuyo3d_driver PRIVATE "HOKUYO3D_BUILDING_DLL")

target_include_directories(hokuyo3d_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(hokuyo3d_driver ${Boost_LIBRARIES})
ament_target_dependencies(hokuyo3d_driver
  rclcpp
  rclcpp_components
  sensor_msgs
  builtin_interfaces
  )
rclcpp_components_register_nodes(hokuyo3d_driver "Hokuyo3d::Hokuyo3dNode")

add_executable(hokuyo3d src/hokuyo3d.cpp)
target_link_libraries(hokuyo3d 
  hokuyo3d_driver
  ${Boost_LIBRARIES})
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
