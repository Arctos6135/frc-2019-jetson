#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ntcore.h>
#include <networktables/NetworkTable.h>

#include <string>

std::string nt_ip_addr;
ros::Subscriber result_sub;

void result_callback(const std_msgs::Float64::ConstPtr &msg) {
	// TODO
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "nt_ros_translate_node");
	
	ros::NodeHandle node_handle("~");

	// Retrieve parameters
	node_handle.param<std::string>("nt_ip_addr", nt_ip_addr, "10.61.35.2");

	// NT setup
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress(nt_ip_addr.c_str());
	NetworkTable::Initialize();

	// Topic subscription
	result_sub = node_handle.subscribe("bot_vision/result_horiz_angle", 1, result_callback);

	return 0;
}

