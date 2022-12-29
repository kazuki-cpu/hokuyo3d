cmake_minimum_required(VERSION 3.5)
project(vssp_debag)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vssp_debag_msgs REQUIRED)
find_package(Boost REQUIRED system)

include_directories(
  include 
  ${Boost_INCLUDE_DIRS}
)

add_executable(vssp_debag src/vssp_debag.cpp)
target_link_libraries(vssp_debag 
  ${Boost_LIBRARIES})
ament_target_dependencies(vssp_debag
  rclcpp
  class_loader
  vsspdebag_msgs)


ament_export_dependencies(rclcpp)
ament_export_dependencies(vsspdebag_msgs)

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS vssp_debag
  DESTINATION lib/${PROJECT_NAME})

ament_package()
