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

#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>

#include <algorithm>
#include <deque>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <vssp.h>

boost::posix_time::time_duration timeout_;

class YVTcommunication
{
	
	
public:

	boost::asio::io_service io_;
  	boost::asio::deadline_timer timer_;
	vssp::VsspDriver driver_;

	YVTcommunication()
	{
	int horizontal_interlace_ = 4;
	int horizontal_interlace_ = 1;
	std::string ip_ = "192.168.11.100";
	int port_ = 10940;

	boost::asio::ip::tcp::socket socket(io_);
	driver_.setTimeout(2.0);
	}	


	void tcp_ip_connect()
	{
		boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip), port);
		timer_.expires_from_now(timeout_);
    		timer_.async_wait(std::bind(&VsspDriver::onTimeoutConnect, driver_, boost::asio::placeholders::error));
    		socket_.async_connect(endpoint, std::bind(&YVTcommunication::vssp_connect, this, boost::asio::placeholders::error));
	}
	
	void vssp_connect()
	{
		driver_.requestPing();
		driver_.setAutoReset(false);
      		driver_.setHorizontalInterlace(horizontal_interlace_);
      		driver_.requestHorizontalTable();
      		driver_.setVerticalInterlace(vertical_interlace_);
      		driver_.requestVerticalTable(vertical_interlace_);
      		driver_.requestData(true, true);
      		driver_.requestAuxData();

		receivePackets()		
     	}
	
	
};//YVTcommunication





 
  


  ~Hokuyo3dNode()
  {
    driver_.requestAuxData(false);
    driver_.requestData(true, false);
    driver_.requestData(false, false);
    driver_.poll();
    ROS_INFO("Communication stoped");
  }
 
  bool poll()
  {
    if (driver_.poll())
    {
      return true;
    }
    ROS_INFO("Connection closed");
    return false;
  }
  void cbTimer(const boost::system::error_code& error)
  {
    if (error)
      return;

    if (!ros::ok())
    {
      driver_.stop();
    }
    else
    {
      timer_.expires_at(
          timer_.expires_at() +
          boost::posix_time::milliseconds(500));
      timer_.async_wait(boost::bind(&Hokuyo3dNode::cbTimer, this, _1));
    }
  }
  void spin()
  {
    timer_.async_wait(boost::bind(&Hokuyo3dNode::cbTimer, this, _1));
    boost::thread thread(
        boost::bind(&boost::asio::io_service::run, &io_));

    ros::AsyncSpinner spinner(1);
    spinner.start();
    driver_.spin();
    spinner.stop();
    timer_.cancel();
    ROS_INFO("Connection closed");
  }
 

protected:
  
  vssp::VsspDriver driver_;
  

 
  bool allow_jump_back_;
  boost::mutex connect_mutex_;

  boost::asio::io_service io_;
  boost::asio::deadline_timer timer_;

  int field_;
  int frame_;
  int line_;


  PublishCycle cycle_;
  std::string ip_;
  int port_;
  int horizontal_interlace_;
  int vertical_interlace_;
  double range_min_;
  std::string frame_id_;

  bool auto_reset_;
  bool set_auto_reset_;
};

int main()
{
	YVTcommunication yvt():

	yvt.tcp_ip_connect()
	yvt.vssp_connect()

	return 1;
}
