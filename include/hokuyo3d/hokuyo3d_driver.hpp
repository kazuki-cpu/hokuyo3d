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
#include <boost/asio/system_timer.hpp>
//#include <deque>
#include <string>//std_msgs/msg/string.hppかも
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp> 
//#include <sensor_msgs/msg/MagneticField.hpp> 


#include <hokuyo3d/vssp.hpp>

namespace Hokuyo3d
{
class Hokuyo3dNode : public rclcpp::Node
{
public:
  HOKUYO3D_PUBLIC 
  explicit Hokuyo3dNode(const rclcpp::NodeOptions & options);
  ~Hokuyo3dNode(void);
  
  void cbPoint(const vssp::Header& header,
      const vssp::RangeHeader& range_header,
      const vssp::RangeIndex& range_index,
      const boost::shared_array<uint16_t>& index,
      const boost::shared_array<vssp::XYZI>& points);
  void cbError(const vssp::Header& header,
      const std::string& message);
  //void cbPing(const vssp::Header& header,
      //const std::chrono::system_clock::time_point& time_read);
  void cbAux(const vssp::Header& header,
      const vssp::AuxHeader& aux_header,
      const boost::shared_array<vssp::Aux>& auxs);
  void cbConnect(bool success);
  //void cbSubscriber();
  bool poll();
  void cbTimer(const boost::system::error_code& error);
  void spin();
  void ping();
  

protected:
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_pc_; 
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc2_; 
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_; 
  //rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_; 
  vssp::VsspDriver driver_;
  sensor_msgs::msg::PointCloud cloud_; 
  sensor_msgs::msg::PointCloud2 cloud2_; 
  sensor_msgs::msg::Imu imu_; 
  //sensor_msgs::msg::MagneticField mag_; 

  bool enable_pc_;
  bool enable_pc2_;
  bool allow_jump_back_;
  std::mutex connect_mutex_;

  //rclcpp::Clock ros_clock_;
  //rclcpp::Time time_ping_; 
  //rclcpp::Time timestamp_base_; 
  //std::deque<rclcpp::Time> timestamp_base_buffer_; 
  builtin_interfaces::msg::Time imu_stamp_last_; 
  //rclcpp::Time mag_stamp_last_; 
  rclcpp::Time pc_stamp;
  rclcpp::Time pc2_stamp;
  builtin_interfaces::msg::Time cloud_stamp_last_; 

  boost::asio::io_service io_;
  boost::asio::system_timer timer_;

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
  //std::string mag_frame_id_;
  bool auto_reset_;
  bool set_auto_reset_;

};
  
} //namespace Hokuyo3d

#endif // HOKUYO3D__HOKUYO3D_DRIVER_HPP_
