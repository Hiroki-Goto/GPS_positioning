#include <ros/ros.h>

#include <boost/asio/serial_port.hpp>
#include <ros/serialization.h>

class Open_port{
	public:

	private:
		
};


int main(int argc, char **argv){
	
	std::string device;
	int baudrate;
	int rate;

	ros::init(argc, argv, "PC");
	Open_port open;
	boost::asio::io_service io_service;
	boost::shared_ptr<boost::asio::serial_port> serial_handle;

	ros::NodeHandle param("~");
	param.param("device", device, std::string("/dev/ttyACM0"));
	param.param("baudrate", baudrate, 115200);
	param.param("rate", rate, 1);	//Hz

	boost::asio::serial_port *serial = new boost::asio::serial_port(io_service);
	serial_handle.reset(serial);

	//open serial port
	try{
		serial -> open(device);
	}catch(std::runtime_error &e){
		ROS_ERROR("Couldn't open device");
		return 1;	//exit
	}

	ROS_INFO("Can open serial port")
	return 0;
}

