#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <ntcore.h>
#include <networktables/NetworkTable.h>

#include <string>
#include <memory>

// NetworkTables IP address
std::string nt_ip_addr;
// Subscriber for receiving vision result
ros::Subscriber result_sub;
// Service client for turning vision on or off
ros::ServiceClient vision_scli;
// The shared table for roboRIO/Jetson comms
std::shared_ptr<NetworkTable> table;
// Callback for when a result is generated
void result_callback(const std_msgs::Float64::ConstPtr &msg) {
	// Put the result into the table
	table->PutNumber("horizontal-angle", msg->data);
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

	table = NetworkTable::GetTable("roborio-jetson");

	// Topic subscription
	result_sub = node_handle.subscribe("bot_vision/result_horiz_angle", 1, result_callback);

	// Service client setup
	vision_scli = node_handle.serviceClient<std_srvs::SetBool>("/vision_processing_node/enable_vision");
	
	return 0;
}

