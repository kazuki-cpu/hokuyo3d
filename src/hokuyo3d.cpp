#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "hokuyo3d/hokuyo3d_driver.hpp"

int main(int argc, char * argv[])//**?
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<Hokuyo3d::Hokuyo3dNode> hokuyo3d_node =
    std::make_shared<Hokuyo3d::Hokuyo3dNode>(options);
  exe.add_node(hokuyo3d_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 1;
}
