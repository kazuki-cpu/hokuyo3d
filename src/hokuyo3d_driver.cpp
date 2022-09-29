/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono> //変更8.15
#include <memory> //追加8.15
#include <thread> //変更9.17
#include <mutex> //変更9.17
#include <boost/asio.hpp>

#include <algorithm>
//#include <deque>
//#include <string>//std_msgs/msg/string.hppかも

#include <rclcpp/rclcpp.hpp> //変更8.15
#include <sensor_msgs/msg/point_cloud2_iterator.hpp> //変更9.17
#include <sensor_msgs/msg/point_cloud_conversion.hpp> //変更9.17

#include <vssp.h>

namespace Hokuyo3d
{

void Hokuyo3dNode::cbPoint(
      const vssp::Header& header,
      const vssp::RangeHeader& range_header,
      const vssp::RangeIndex& range_index,
      const boost::shared_array<uint16_t>& index,
      const boost::shared_array<vssp::XYZI>& points,
      const boost::posix_time::ptime& time_read)
  {
    if (timestamp_base_ == rclcpp::Time(0))//変更9.25
      return;
    // Pack scan data
    if (enable_pc_)
    {
      if (cloud_.points.size() == 0)
      {
        // Start packing PointCloud message
        cloud_.header.frame_id = frame_id_;
        cloud_.header.stamp = timestamp_base_ + rclcpp::Duration(range_header.line_head_timestamp_ms * 0.001);
      }
      // Pack PointCloud message
      for (int i = 0; i < index[range_index.nspots]; i++)
      {
        if (points[i].r < range_min_)
        {
          continue;
        }
        geometry_msgs::msg::Point32 point;//変更9.17
        point.x = points[i].x;
        point.y = points[i].y;
        point.z = points[i].z;
        cloud_.points.push_back(point);
        cloud_.channels[0].values.push_back(points[i].i);
      }
    }
    if (enable_pc2_)
    {
      if (cloud2_.data.size() == 0)
      {
        // Start packing PointCloud2 message
        cloud2_.header.frame_id = frame_id_;
        cloud2_.header.stamp = timestamp_base_ + rclcpp::Duration(range_header.line_head_timestamp_ms * 0.001);
        cloud2_.row_step = 0;
        cloud2_.width = 0;
      }
      // Pack PointCloud2 message
      cloud2_.data.resize((cloud2_.width + index[range_index.nspots]) * cloud2_.point_step);

      float* data = reinterpret_cast<float*>(&cloud2_.data[0]);
      data += cloud2_.width * cloud2_.point_step / sizeof(float);
      for (int i = 0; i < index[range_index.nspots]; i++)
      {
        if (points[i].r < range_min_)
        {
          continue;
        }
        *(data++) = points[i].x;
        *(data++) = points[i].y;
        *(data++) = points[i].z;
        *(data++) = points[i].i;
        cloud2_.width++;
      }
      cloud2_.row_step = cloud2_.width * cloud2_.point_step;
    }
    // Publish points
    if ((cycle_ == CYCLE_FIELD && (range_header.field != field_ || range_header.frame != frame_)) ||
        (cycle_ == CYCLE_FRAME && (range_header.frame != frame_)) || (cycle_ == CYCLE_LINE))
    {
      if (enable_pc_)
      {
        if (cloud_.header.stamp < cloud_stamp_last_ && !allow_jump_back_)
        {
          ROS_INFO("Dropping timestamp jump backed cloud");
        }
        else
        {
          pub_pc_->publish(cloud_);//9.2変更
        }
        cloud_stamp_last_ = cloud_.header.stamp;
        cloud_.points.clear();
        cloud_.channels[0].values.clear();
      }
      if (enable_pc2_)
      {
        cloud2_.data.resize(cloud2_.width * cloud2_.point_step);
        if (cloud2_.header.stamp < cloud_stamp_last_ && !allow_jump_back_)
        {
          ROS_INFO("Dropping timestamp jump backed cloud2");
        }
        else
        {
          pub_pc2_->publish(cloud2_);//9.2変更
        }
        cloud_stamp_last_ = cloud2_.header.stamp;
        cloud2_.data.clear();
      }
      if (range_header.frame != frame_)
        ping();
      frame_ = range_header.frame;
      field_ = range_header.field;
      line_ = range_header.line;
    }
  }
  void Hokuyo3dNode::cbError(
      const vssp::Header& header,
      const std::string& message,
      const boost::posix_time::ptime& time_read)
  {
    ROS_ERROR("%s", message.c_str());
  }
  void Hokuyo3dNode::cbPing(
      const vssp::Header& header,
      const boost::posix_time::ptime& time_read)
  {
    const rclcpp::Time now = ros::Time::fromBoost(time_read);//一部変更9.17
    const ros::Duration delay =
        ((now - time_ping_) - ros::Duration(header.send_time_ms * 0.001 - header.received_time_ms * 0.001)) * 0.5;
    const rclcpp::Time base = time_ping_ + delay - ros::Duration(header.received_time_ms * 0.001);//一部変更9.17

    timestamp_base_buffer_.push_back(base);
    if (timestamp_base_buffer_.size() > 5)
      timestamp_base_buffer_.pop_front();

    auto sorted_timstamp_base = timestamp_base_buffer_;
    std::sort(sorted_timstamp_base.begin(), sorted_timstamp_base.end());

    if (timestamp_base_ == ros::Time(0))
      timestamp_base_ = sorted_timstamp_base[sorted_timstamp_base.size() / 2];
    else
      timestamp_base_ += (sorted_timstamp_base[sorted_timstamp_base.size() / 2] - timestamp_base_) * 0.1;

    ROS_DEBUG("timestamp_base: %lf", timestamp_base_.toSec());
  }
  void Hokuyo3dNode::cbAux(
      const vssp::Header& header,
      const vssp::AuxHeader& aux_header,
      const boost::shared_array<vssp::Aux>& auxs,
      const boost::posix_time::ptime& time_read)
  {
    if (timestamp_base_ == ros::Time(0))
      return;
    rclcpp::Time stamp = timestamp_base_ + ros::Duration(aux_header.timestamp_ms * 0.001);//一部変更9.17

    if ((aux_header.data_bitfield & (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC)) ==
        (vssp::AX_MASK_ANGVEL | vssp::AX_MASK_LINACC))
    {
      imu_.header.frame_id = imu_frame_id_;
      imu_.header.stamp = stamp;
      for (int i = 0; i < aux_header.data_count; i++)
      {
        imu_.orientation_covariance[0] = -1.0;
        imu_.angular_velocity.x = auxs[i].ang_vel.x;
        imu_.angular_velocity.y = auxs[i].ang_vel.y;
        imu_.angular_velocity.z = auxs[i].ang_vel.z;
        imu_.linear_acceleration.x = auxs[i].lin_acc.x;
        imu_.linear_acceleration.y = auxs[i].lin_acc.y;
        imu_.linear_acceleration.z = auxs[i].lin_acc.z;
        if (imu_stamp_last_ > imu_.header.stamp && !allow_jump_back_)
        {
          ROS_INFO("Dropping timestamp jump backed imu");
        }
        else
        {
          pub_imu_.publish(imu_);
        }
        imu_stamp_last_ = imu_.header.stamp;
        imu_.header.stamp += ros::Duration(aux_header.data_ms * 0.001);
      }
    }
    if ((aux_header.data_bitfield & vssp::AX_MASK_MAG) == vssp::AX_MASK_MAG)
    {
      mag_.header.frame_id = mag_frame_id_;
      mag_.header.stamp = stamp;
      for (int i = 0; i < aux_header.data_count; i++)
      {
        mag_.magnetic_field.x = auxs[i].mag.x;
        mag_.magnetic_field.y = auxs[i].mag.y;
        mag_.magnetic_field.z = auxs[i].mag.z;
        if (mag_stamp_last_ > imu_.header.stamp && !allow_jump_back_)
        {
          ROS_INFO("Dropping timestamp jump backed mag");
        }
        else
        {
          pub_mag_.publish(mag_);
        }
        mag_stamp_last_ = imu_.header.stamp;
        mag_.header.stamp += ros::Duration(aux_header.data_ms * 0.001);
      }
    }
  }
  void Hokuyo3dNode::cbConnect(bool success)
  {
    if (success)
    {
      ROS_INFO("Connection established");
      ping();
      if (set_auto_reset_)
        driver_.setAutoReset(auto_reset_);
      driver_.setHorizontalInterlace(horizontal_interlace_);
      driver_.requestHorizontalTable();
      driver_.setVerticalInterlace(vertical_interlace_);
      driver_.requestVerticalTable(vertical_interlace_);
      driver_.requestData(true, true);
      driver_.requestAuxData();
      driver_.receivePackets();
      ROS_INFO("Communication started");
    }
    else
    {
      ROS_ERROR("Connection failed");
    }
  }
  Hokuyo3dNode::Hokuyo3dNode(const rclcpp::NodeOptions & options)
  : Node("hokuyo3d", options)
    , timestamp_base_(0)
    , timer_(io_, boost::posix_time::milliseconds(500))
  {

    horizontal_interlace_ = this->declare_parameter<int>("horizontal_interlace", 4);//変更8.17
    vertical_interlace_ = this->declare_parameter<int>("vertical_interlace", 1);//変更8.17
    ip_ = this->declare_parameter<std::string>("ip", "192.168.0.10");//変更8.17
    port_ = this->declare_parameter<int>("port", 10940);//変更8.17
    frame_id_ = this->declare_parameter<std::string>("frame_id", "hokuyo3d");//変更8.17
    imu_frame_id_ = this->declare_parameter<std::string>("imu_frame_id", frame_id_ + "_imu");//変更8.17
    mag_frame_id_ = this->declare_parameter<std::string>("mag_frame_id", frame_id_ + "_mag");//変更8.17
    range_min_ = this->declare_parameter<double>("range_min", 0.0);//変更8.17
    set_auto_reset_ = true;
    auto_reset_ = this->declare_parameter<bool>("auto_reset", false);//変更8.17
    allow_jump_back_ = this->declare_parameter<bool>("allow_jump_back", false);//変更8.17

    std::string output_cycle;
    output_cycle = this->declare_parameter<std::string>("output_cycle", "field");//変更8.17
        

    if (output_cycle.compare("frame") == 0)
      cycle_ = CYCLE_FRAME;
    else if (output_cycle.compare("field") == 0)
      cycle_ = CYCLE_FIELD;
    else if (output_cycle.compare("line") == 0)
      cycle_ = CYCLE_LINE;
    else
    {
      ROS_ERROR("Unknown output_cycle value %s", output_cycle.c_str());
      rclcpp::shutdown();
    }

    driver_.setTimeout(2.0);
    ROS_INFO("Connecting to %s", ip_.c_str());
    driver_.registerCallback(std::bind(&Hokuyo3dNode::cbPoint, this, _1, _2, _3, _4, _5, _6));//変更9.17
    driver_.registerAuxCallback(std::bind(&Hokuyo3dNode::cbAux, this, _1, _2, _3, _4));//変更9.17
    driver_.registerPingCallback(std::bind(&Hokuyo3dNode::cbPing, this, _1, _2));//変更9.17
    driver_.registerErrorCallback(std::bind(&Hokuyo3dNode::cbError, this, _1, _2, _3));//変更9.17
    field_ = 0;
    frame_ = 0;
    line_ = 0;

    sensor_msgs::msg::ChannelFloat32 channel;
    channel.name = std::string("intensity");
    cloud_.channels.push_back(channel);

    cloud2_.height = 1;
    cloud2_.is_bigendian = false;
    cloud2_.is_dense = false;
    sensor_msgs::msg::PointCloud2Modifier pc2_modifier(cloud2_);//この関数は::msg::でもちゃんと動く？
    pc2_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 5);//更新9.17(9.2)
    pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 5);//更新9.17(9.2)

    enable_pc_ = enable_pc2_ = true;//
    //ros::SubscriberStatusCallback cb_con = std::bind(&Hokuyo3dNode::cbSubscriber, this);//一部変更9.17

    std::lock_guard<std::mutex> lock(connect_mutex_);//変更9.17
    pub_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud>("hokuyo_cloud", 5);//更新9.17(9.2)
    pub_pc2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("hokuyo_cloud2", 5);//更新9.17(9.2)

    // Start communication with the sensor
    driver_.connect(ip_.c_str(), port_, std::bind(&Hokuyo3dNode::cbConnect, this, _1));//変更9.17
  }
  Hokuyo3dNode::~Hokuyo3dNode()
  {
    driver_.requestAuxData(false);
    driver_.requestData(true, false);
    driver_.requestData(false, false);
    driver_.poll();
    ROS_INFO("Communication stoped");
  }
/*  void Hokuyo3dNode::cbSubscriber()
  {
    std::lock_guard<std::mutex> lock(connect_mutex_);//変更9.17
    if (pub_pc_.getNumSubscribers() > 0)
    {
      enable_pc_ = true;
      ROS_DEBUG("PointCloud output enabled");
    }
    else
    {
      enable_pc_ = false;
      ROS_DEBUG("PointCloud output disabled");
    }
    if (pub_pc2_.getNumSubscribers() > 0)
    {
      enable_pc2_ = true;
      ROS_DEBUG("PointCloud2 output enabled");
    }
    else
    {
      enable_pc2_ = false;
      ROS_DEBUG("PointCloud2 output disabled");
    }
  }
*/
  bool Hokuyo3dNode::poll()
  {
    if (driver_.poll())
    {
      return true;
    }
    ROS_INFO("Connection closed");
    return false;
  }
  void Hokuyo3dNode::cbTimer(const boost::system::error_code& error)
  {
    if (error)
      return;

    if (!rclcpp::ok())//変更9.17
    {
      driver_.stop();
    }
    else
    {
      timer_.expires_at(
          timer_.expires_at() +
          boost::posix_time::milliseconds(500));
      timer_.async_wait(std::bind(&Hokuyo3dNode::cbTimer, this, _1));//変更9.17
    }
  }
  void Hokuyo3dNode::spin()
  {
    timer_.async_wait(std::bind(&Hokuyo3dNode::cbTimer, this, _1));//変更9.17
    std::thread thread(//変更9.17
        std::bind(&boost::asio::io_service::run, &io_));//変更9.17

    ros::AsyncSpinner spinner(1);
    spinner.start();
    driver_.spin();
    spinner.stop();
    timer_.cancel();
    ROS_INFO("Connection closed");
  }
  void Hokuyo3dNode::ping()
  {
    driver_.requestPing();
    time_ping_ = ros::Time::now();
  }
 
};

/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "hokuyo3d");
  Hokuyo3dNode node;

  node.spin();

  return 1;
}
*/

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Hokuyo3d::Hokuyo3dNode)
