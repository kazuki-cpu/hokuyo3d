#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <mutex> 
#include <thread> 

#include <boost/asio.hpp>
#include <boost/asio/system_timer.hpp>
#include <vsspdebag/vssp.hpp>

#include <vsspdebag_msgs/msg/aux_header.hpp>
#include <vsspdebag_msgs/msg/header.hpp>
#include <vsspdebag_msgs/msg/range_header.hpp>

class YVTcommunication: public rclcpp::Node
{

public:

	YVTcommunication(
		const std::string & node_name,
		const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
	): rclcpp::Node{node_name, options}
	  ,timer_(io_, std::chrono::milliseconds(500))
	{
		int horizontal_interlace_ = 4;
		int vertical_interlace_ = 1;
		std::string ip_ = "192.168.11.100";
		int port_ = 10940;	
		
		//publisher establish
		header_pub = this->create_publisher<vsspdebag_msgs::msg::Header>("header", 10);
		range_header_pub = this->create_publisher<vsspdebag_msgs::msg::RangeHeader>("range_header", 10);
		aux_header_pub = this->create_publisher<vsspdebag_msgs::msg::AuxHeader>("aux_header", 10);
		
		//register callback function
		driver_.registerHeaderCallback(std::bind(&YVTcommunication::Header_publish, this, _1));
		driver_.registerRangeHeaderCallback(std::bind(&YVTcommunication::RangeHeader_publish, this, _1));
		driver_.registerAuxHeaderCallback(std::bind(&YVTcommunication::AuxHeader_publish, this, _1));
  
  		std::lock_guard<std::mutex> lock(connect_mutex_);
  		tcp_ip_connect(ip_, port_);
		spin();
	}
	
	~YVTcommunication()
  	{
    		driver_.requestAuxData(false);
    		driver_.requestData(true, false);
    		driver_.requestData(false, false);
    		driver_.poll();
    		RCLCPP_INFO("Communication stoped");
  	}

	void tcp_ip_connect(const char* ip, const unsigned int port)
	{
		boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip), port);
		timer_.expires_from_now(std::chrono::milliseconds(2000));
    		timer_.async_wait(std::bind(&VsspDriver::onTimeoutConnect, driver_, boost::asio::placeholders::error));
    		socket_.async_connect(endpoint, std::bind(&YVTcommunication::vssp_connect, this, boost::asio::placeholders::error));
	}
	void vssp_connect()
	{
		timer_.cancel();
		driver_.requestPing();
		driver_.setAutoReset(false);
      		driver_.setHorizontalInterlace(horizontal_interlace_);
      		driver_.requestHorizontalTable();
      		driver_.setVerticalInterlace(vertical_interlace_);
      		driver_.requestVerticalTable(vertical_interlace_);
      		driver_.requestData(true, true);
      		driver_.requestAuxData();
		driver_.receivePackets();		
     	}
	void cbTimer(const boost::system::error_code& error)
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
          			std::chrono::milliseconds(500));
      			timer_.async_wait(std::bind(&YVTcommunication::cbTimer, this, _1));
    		}
  	}
  	void spin()
  	{
    		timer_.async_wait(std::bind(&YVTcommunication::cbTimer, this, _1));
    		std::thread thread(
        	std::bind(
  			static_cast<std::size_t (boost::asio::io_service::*)()>(&boost::asio::io_service::run), &io_service_));

    		driver_.spin();
    		timer_.cancel();
    		RCLCPP_INFO(get_logger(),"Connection closed");
  	}
	void Header_publish(const vssp::Header& header)
  	{
    		header_.mark = header.mark;
    		header_.type = header.type;
    		header_.status = header.status;
    		header_.header_length = header.header_length;
    		header_.length = header.length;
    		header_.received_time_ms = header.received_time_ms;
    		header_.send_time_ms = header.send_time_ms;
    		header_pub->publish(header_);
  	}
  	void RangeHeader_publish(const vssp::RangeHeader& range_header, 
				 const vssp::RangeHeaderV2R1& range_header_v2r1)
  	{
    		range_header_.header_length = range_header.header_length;
    		range_header_.line_head_timestamp_ms = range_header.line_head_timestamp_ms;
    		range_header_.line_tail_timestamp_ms = range_header.line_tail_timestamp_ms;
    		range_header_.line_head_h_angle_ratio = range_header.line_head_h_angle_ratio;
    		range_header_.line_tail_h_angle_ratio = range_header.line_tail_h_angle_ratio;
    		range_header_.frame = range_header.frame;
    		range_header_.field = range_header.field;
    		range_header_.line = range_header.line;
    		range_header_.spot = range_header.spot;
    		range_header_.vertical_field = range_header_v2r1.vertical_field;
    		range_header_.vertical_interlace = range_header_v2r1.vertical_interlace;
    		range_header_pub->publish(range_header_);
  	}
  	void AuxHeader_publish(const vssp::AuxHeader& aux_header)
  	{
    		aux_header_.header_length = aux_header.header_length;
    		aux_header_.timestamp_ms = aux_header.timestamp_ms;
    		aux_header_.data_bitfield = aux_header.data_bitfield;
    		aux_header_.data_count = aux_header.data_count;
    		aux_header_.data_ms = aux_header.data_ms;
    		aux_header_pub->publish(aux_header_);
  	}

protected:

   vssp::VsspDriver driver_;
   std::mutex connect_mutex_;
   
   vsspdebag_msgs::msg::Header header_;
   vsspdebag_msgs::msg::RangeHeader range_header_;
   vsspdebag_msgs::msg::AuxHeader aux_header_;
  
   rclcpp::Publisher<vsspdebag_msgs::msg::Header>::SharedPtr header_pub;
   rclcpp::Publisher<vsspdebag_msgs::msg::RangeHeader>::SharedPtr range_header_pub;
   rclcpp::Publisher<vsspdebag_msgs::msg::AuxHeader>::SharedPtr aux_header_pub;
	
};//YVTcommunication

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	
	auto node = rclcpp::Node::make_shared<YVTcommunication>("vsspdebag");
	
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	
	rclcpp::shutdown();
	return 0;
}
