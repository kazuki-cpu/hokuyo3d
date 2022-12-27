#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>

#include <boost/asio.hpp>
#include <vssp.hpp>

#include <vssp_debag_msgs/msg/Aux.hpp>
#include <vssp_debag_msgs/msg/AuxHeader.hpp>
#include <vssp_debag_msgs/msg/Header.hpp>
#include <vssp_debag_msgs/msg/RangeHeader.hpp>
#include <vssp_debag_msgs/msg/XYZI.hpp>

std::chrono::milliseconds timeout_;

class YVTcommunication: public rclcpp::Node
{		
public:
	boost::asio::io_service io_;
  	boost::asio::system_timer timer_;
	vssp::VsspDriver driver_;
	
	vssp_debag_msgs::msg::Header header_;
	vssp_debag_msgs::msg::RangeHeader range_header_;
	vssp_debag_msgs::msg::AuxHeader aux_header_;
	vssp_debag_msgs::msg::Aux aux_;
	vssp_debag_msgs::msg::XYZI xyzi_;
	
	rclcpp::Publisher<vssp_debag_msgs::msg::Header>::SharedPtr header_pub;
	rclcpp::Publisher<vssp_debag_msgs::msg::RangeHeader>::SharedPtr range_header_pub;
	rclcpp::Publisher<vssp_debag_msgs::msg::AuxHeader>::SharedPtr aux_header_pub;
	rclcpp::Publisher<vssp_debag_msgs::msg::Aux>::SharedPtr aux_pub;
	rclcpp::Publisher<vssp_debag_msgs::msg::XYZI>::SharedPtr xyzi_pub;
	

	YVTcommunication(
		const std::string & node_name,
		const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
	): rclcpp::Node{node_name, options}
	  ,timer_(io_, std::chrono::milliseconds(500))
	{
		int horizontal_interlace_ = 4;
		int horizontal_interlace_ = 1;
		std::string ip_ = "192.168.11.100";
		int port_ = 10940;

		boost::asio::ip::tcp::socket socket(io_);
		
		header_pub = this->create_publisher<vssp_debag_msgs::msg::Header>("header", 10);
		range_header_pub = this->create_publisher<vssp_debag_msgs::msg::RangeHeader>("range_header", 10);
		aux_header_pub = this->create_publisher<vssp_debag_msgs::msg::AuxHeader>("aux_header", 10);
		aux_pub = this->create_publisher<vssp_debag_msgs::msg::Aux>("aux", 10);
		xyzi_pub = this->create_publisher<vssp_debag_msgs::msg::XYZI>("xyzi", 10);
		
		driver_.setTimeout(2.0);
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
		timer_.expires_from_now(timeout_);
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
      			timer_.async_wait(std::bind(&Hokuyo3dNode::cbTimer, this, _1));
    		}
  	}
	
  	void spin()
  	{
    		timer_.async_wait(std::bind(&Hokuyo3dNode::cbTimer, this, _1));
    		std::thread thread(
        	std::bind(
  			static_cast<std::size_t (boost::asio::io_service::*)()>(&boost::asio::io_service::run), &io_));

    		driver_.spin();
    		timer_.cancel();
    		RCLCPP_INFO(get_logger(),"Connection closed");
  	}
	
};//YVTcommunication

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	
	auto node = rclcpp::Node::make_shared<YVTcommunication>("vssp_debag");
	
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	
	rclcpp::shutdown();
	return 0;
}
