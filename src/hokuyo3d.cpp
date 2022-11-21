//#include <chrono>
#include <boost/asio.hpp>
//#include <string>
#include <vssp.h>

boost::posix_time::time_duration timeout_;

class YVTcommunication{
	
	
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
	
	
};//YVTcommunication

int main()
{
	YVTcommunication yvt():

	yvt.tcp_ip_connect()
	

	return 1;
}
