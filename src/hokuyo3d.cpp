#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "hokuyo3d/hokuyo3d_driver.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto greeter = std::make_shared<greeter_ros2_style::Greeter>(options);
  rclcpp::spin(greeter);

  rclcpp::shutdown();
  return 0;
}

/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hokuyo3d");
  Hokuyo3dNode node;

  node.spin();

  return 1;
}
*/
