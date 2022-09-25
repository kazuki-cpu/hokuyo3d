#ifndef HOKUYO3D__HOKUYO3D_DRIVER_HPP_ 
#define HOKUYO3D__HOKUYO3D_DRIVER_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HOKUYO3D_EXPORT __attribute__ ((dllexport))
    #define HOKUYO3D_IMPORT __attribute__ ((dllimport))
  #else
    #define HOKUYO3D_EXPORT __declspec(dllexport)
    #define HOKUYO3D_IMPORT __declspec(dllimport)
  #endif
  #ifdef HOKUYO3D_BUILDING_DLL
    #define HOKUYO3D_PUBLIC HOKUYO3D_EXPORT
  #else
    #define HOKUYO3D_PUBLIC HOKUYO3D_IMPORT
  #endif
  #define HOKUYO3D_PUBLIC_TYPE HOKUYO3D_PUBLIC
  #define HOKUYO3D_LOCAL
#else
  #define HOKUYO3D_EXPORT __attribute__ ((visibility("default")))
  #define HOKUYO3D_IMPORT
  #if __GNUC__ >= 4
    #define HOKUYO3D_PUBLIC __attribute__ ((visibility("default")))
    #define HOKUYO3D_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HOKUYO3D_PUBLIC
    #define HOKUYO3D_LOCAL
  #endif
  #define HOKUYO3D_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <mutex> 
#include <boost/asio.hpp>
#include <deque>
#include <string>//std_msgs/msg/string.hppかも
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/PointCloud.hpp> //変更9.17
#include <sensor_msgs/msg/PointCloud2.hpp> //変更9.17
#include <sensor_msgs/msg/Imu.hpp> //変更9.17
#include <sensor_msgs/msg/MagneticField.hpp> //変更9.17


#include <vssp.h>

namespace Hokuyo3d
{
class Hokuyo3dNode : public rclcpp::Node
{
public:
  HOKUYO3D_PUBLIC explicit Hokuyo3dNode(const rclcpp::NodeOptions & options);
  
  void cbPoint();
  void cbError();
  void cbPing();
  void cbAux();
  void cbConnect();
  //void cbSubscriber();
  bool poll();
  void cbTimer();
  void spin();
  void ping();
  

protected:
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>SharedPtr pub_pc_; //更新9.17(9.2)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>SharedPtr pub_pc2_; //更新9.17(9.2)
  rclcpp::Publisher<sensor_msgs::msg::Imu>SharedPtr pub_imu_; //更新9.17(9.2)
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>SharedPtr pub_mag_; //更新9.17(9.2)
  vssp::VsspDriver driver_;
  sensor_msgs::msg::PointCloud cloud_; //変更9.17
  sensor_msgs::msg::PointCloud2 cloud2_; //変更9.17
  sensor_msgs::msg::Imu imu_; //変更9.17
  sensor_msgs::msg::MagneticField mag_; //変更9.17

  bool enable_pc_;
  bool enable_pc2_;
  bool allow_jump_back_;
  std::mutex connect_mutex_;//変更9.17

  rclcpp::Time time_ping_; //変更9.17
  rclcpp::Time timestamp_base_; //変更9.17
  std::deque<rclcpp::Time> timestamp_base_buffer_; //変更9.17
  rclcpp::Time imu_stamp_last_; //変更9.17
  rclcpp::Time mag_stamp_last_; //変更9.17
  rclcpp::Time cloud_stamp_last_; //変更9.17

  boost::asio::io_service io_;
  boost::asio::deadline_timer timer_;

  int field_;
  int frame_;
  int line_;

  enum PublishCycle
  {
    CYCLE_FIELD,
    CYCLE_FRAME,
    CYCLE_LINE
  };
  PublishCycle cycle_;
  std::string ip_;
  int port_;
  int horizontal_interlace_;
  int vertical_interlace_;
  double range_min_;
  std::string frame_id_;
  std::string imu_frame_id_;
  std::string mag_frame_id_;
  bool auto_reset_;
  bool set_auto_reset_;


};
  
} //namespace Hokuyo3d

#endif // HOKUYO3D__HOKUYO3D_DRIVER_HPP_
