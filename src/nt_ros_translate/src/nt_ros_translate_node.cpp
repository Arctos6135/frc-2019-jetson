#include <ros/ros.h>
#include <ntcore.h>
#include <networktables/NetworkTable.h>

#include <string>

std::string nt_ip_addr;
ros::Subscriber result_sub;

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "nt_ros_translate_node");
	
	ros::NodeHandle node_handle("~");

	// Retrieve parameters
	node_handle.param<std::string>("nt_ip_addr", nt_ip_addr, "10.61.35.2");

	// NT setup
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress(nt_ip_addr.c_str());
	NetworkTable::Initialize();

	return 0;
}

