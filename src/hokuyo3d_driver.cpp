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

#include <chrono> 
#include <memory> 
#include <thread> 
#include <mutex> 
#include <boost/asio.hpp>

#include <algorithm>
#include <deque>
#include <string>

#include <rclcpp/rclcpp.hpp> 
//#include <sensor_msgs/msg/point_cloud2_iterator.hpp> 
//#include <sensor_msgs/msg/point_cloud_conversion.hpp> 

#include <hokuyo3d/hokuyo3d_driver.hpp>
#include <hokuyo3d/vssp.hpp>

using namespace std::placeholders
using namespace std::chrono//12/2追加

namespace Hokuyo3d
{

void Hokuyo3dNode::cbPoint(
      const vssp::Header& header,
      const vssp::RangeHeader& range_header,
      const vssp::RangeIndex& range_index,
      const boost::shared_array<uint16_t>& index,
      const boost::shared_array<vssp::XYZI>& points,
      const system_clock::time_point& time_read)//12/2変更
  {
    if (timestamp_base_ == rclcpp::Time(0))
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
        geometry_msgs::msg::Point32 point;
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
          RCLCPP_INFO(get_logger(), "Dropping timestamp jump backed cloud");
        }
        else
        {
          pub_pc_->publish(cloud_);
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
          RCLCPP_INFO(get_logger(), "Dropping timestamp jump backed cloud2");
        }
        else
        {
          pub_pc2_->publish(cloud2_);
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
      const system_clock::time_point& time_read)
  {
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }
  void Hokuyo3dNode::cbPing(
      const vssp::Header& header,
      const system_clock::time_point& time_read)
  {
    
    seconds total_s = duration_cast<seconds>(time_read.time_since_epoch());
    microseconds micro_s = duration_cast<microseconds>(time_read.time_since_epoch());
    system_clock::duration fractional_s = micro_s - total_s;

    rclcpp::Time now;
    now.seconds = total_s;
    now.nanoseconds = fractional_s / 1000;

    const rclcpp::Duration delay =
        ((now - time_ping_) - rclcpp::Duration(header.send_time_ms * 0.001 - header.received_time_ms * 0.001)) * 0.5;
    const rclcpp::Time base = time_ping_ + delay - rclcpp::Duration(header.received_time_ms * 0.001);

    timestamp_base_buffer_.push_back(base);
    if (timestamp_base_buffer_.size() > 5)
      timestamp_base_buffer_.pop_front();

    auto sorted_timstamp_base = timestamp_base_buffer_;
    std::sort(sorted_timstamp_base.begin(), sorted_timstamp_base.end());

    if (timestamp_base_ == rclcpp::Time(0))
      timestamp_base_ = sorted_timstamp_base[sorted_timstamp_base.size() / 2];
    else
      timestamp_base_ += (sorted_timstamp_base[sorted_timstamp_base.size() / 2] - timestamp_base_) * 0.1;

    RCLCPP_DEBUG(get_logger(), "timestamp_base: %lf", timestamp_base_.seconds);//12/3変更
  }
  void Hokuyo3dNode::cbAux(
      const vssp::Header& header,
      const vssp::AuxHeader& aux_header,
      const boost::shared_array<vssp::Aux>& auxs,
      const system_clock::time_point& time_read)
  {
    if (timestamp_base_ == rclcpp::Time(0))
      return;
    rclcpp::Time stamp = timestamp_base_ + rclcpp::Duration(aux_header.timestamp_ms * 0.001);

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
          RCLCPP_INFO(get_logger(), "Dropping timestamp jump backed imu");
        }
        else
        {
          pub_imu_->publish(imu_);
        }
        imu_stamp_last_ = imu_.header.stamp;
        imu_.header.stamp += rclcpp::Duration(aux_header.data_ms * 0.001);
      }
    }
    
    /*
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
        mag_.header.stamp += rclcpp::Duration(aux_header.data_ms * 0.001);
      }
    }
    */
  }
  void Hokuyo3dNode::cbConnect(bool success)
  {
    if (success)
    {
      RCLCPP_INFO(get_logger(), "Connection established");
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
      RCLCPP_INFO(get_logger(), "Communication started");
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Connection failed");
    }
  }
  Hokuyo3dNode::Hokuyo3dNode(const rclcpp::NodeOptions & options)
  : Node("hokuyo3d", options)
    , timestamp_base_(0)
    , timer_(io_, milliseconds(500))//std::chrono::
  {

    horizontal_interlace_ = this->declare_parameter<int>("horizontal_interlace", 4);
    vertical_interlace_ = this->declare_parameter<int>("vertical_interlace", 1);
    ip_ = this->declare_parameter<std::string>("ip", "192.168.0.10");
    port_ = this->declare_parameter<int>("port", 10940);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "hokuyo3d");
    imu_frame_id_ = this->declare_parameter<std::string>("imu_frame_id", frame_id_ + "_imu");
    //mag_frame_id_ = this->declare_parameter<std::string>("mag_frame_id", frame_id_ + "_mag");
    range_min_ = this->declare_parameter<double>("range_min", 0.0);
    set_auto_reset_ = true;
    auto_reset_ = this->declare_parameter<bool>("auto_reset", false);
    allow_jump_back_ = this->declare_parameter<bool>("allow_jump_back", false);

    std::string output_cycle;
    output_cycle = this->declare_parameter<std::string>("output_cycle", "field");
        
    if (output_cycle.compare("frame") == 0)
      cycle_ = CYCLE_FRAME;
    else if (output_cycle.compare("field") == 0)
      cycle_ = CYCLE_FIELD;
    else if (output_cycle.compare("line") == 0)
      cycle_ = CYCLE_LINE;
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown output_cycle value %s", output_cycle.c_str());
      rclcpp::shutdown();
    }

    driver_.setTimeout(2.0);
    RCLCPP_INFO(this->get_logger(), "Connecting to %s", ip_.c_str());
    driver_.registerCallback(std::bind(&Hokuyo3dNode::cbPoint, this, _1, _2, _3, _4, _5, _6));
    driver_.registerAuxCallback(std::bind(&Hokuyo3dNode::cbAux, this, _1, _2, _3, _4));
    driver_.registerPingCallback(std::bind(&Hokuyo3dNode::cbPing, this, _1, _2));
    driver_.registerErrorCallback(std::bind(&Hokuyo3dNode::cbError, this, _1, _2, _3));
    field_ = 0;
    frame_ = 0;
    line_ = 0;

    sensor_msgs::msg::ChannelFloat32 channel;
    channel.name = std::string("intensity");
    cloud_.channels.push_back(channel);

    cloud2_.height = 1;
    cloud2_.is_bigendian = false;
    cloud2_.is_dense = false;
    
    cloud2_.fields[0].name = "x";//12/2変更
    cloud2_.fields[0].offset = 0;
    cloud2_.fields[0].datatype = 7;
    cloud2_.fields[0].count = 1;
    cloud2_.fields[1].name = "y";//12/2変更
    cloud2_.fields[1].offset = 4;
    cloud2_.fields[1].datatype = 7;
    cloud2_.fields[1].count = 1;    
    cloud2_.fields[2].name = "z";//12/2変更
    cloud2_.fields[2].offset = 8;
    cloud2_.fields[2].datatype = 7;
    cloud2_.fields[2].count = 1;
    cloud2_.fields[3].name = "intensity";//12/2変更
    cloud2_.fields[3].offset = 12;
    cloud2_.fields[3].datatype = 7;
    cloud2_.fields[3].count = 1;
  
    /*
    sensor_msgs::msg::PointCloud2Modifier pc2_modifier(cloud2_);
    pc2_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    */

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 5);

    enable_pc_ = enable_pc2_ = true;
    //ros::SubscriberStatusCallback cb_con = std::bind(&Hokuyo3dNode::cbSubscriber, this);

    std::lock_guard<std::mutex> lock(connect_mutex_);//変更9.17
    pub_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud>("hokuyo_cloud", 5);
    pub_pc2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("hokuyo_cloud2", 5);

    // Start communication with the sensor
    driver_.connect(ip_.c_str(), port_, std::bind(&Hokuyo3dNode::cbConnect, this, _1));
    spin();
  }
  Hokuyo3dNode::~Hokuyo3dNode()
  {
    driver_.requestAuxData(false);
    driver_.requestData(true, false);
    driver_.requestData(false, false);
    driver_.poll();
    RCLCPP_INFO(this->get_logger(), "Communication stoped");
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
      RCLCPP_DEBUG("PointCloud2 output disabled");
    }
  }
*/

  bool Hokuyo3dNode::poll()
  {
    if (driver_.poll())
    {
      return true;
    }
    RCLCPP_INFO(get_logger(), "Connection closed");
    return false;
  }
  void Hokuyo3dNode::cbTimer(const boost::system::error_code& error)
  {
    if (error)
      return;

    if (!rclcpp::ok())
    {
      driver_.stop();
    }
    else
    {
      timer_.expires_at(
          timer_.expires_at() +
          milliseconds(500));//std::chrono::
      timer_.async_wait(std::bind(&Hokuyo3dNode::cbTimer, this, _1));
    }
  }
  void Hokuyo3dNode::spin()
  {
    timer_.async_wait(std::bind(&Hokuyo3dNode::cbTimer, this, _1));
    std::thread thread(
        std::bind(
  static_cast<std::size_t (boost::asio::io_service::*)()>(&boost::asio::io_service::run), &io_));//12/2変更

    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    driver_.spin();
    //spinner.stop();
    timer_.cancel();
    RCLCPP_INFO(get_logger(),"Connection closed");
  }
  void Hokuyo3dNode::ping()
  {
    driver_.requestPing();
    time_ping_ = this->now();
  }
 
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Hokuyo3d::Hokuyo3dNode)
