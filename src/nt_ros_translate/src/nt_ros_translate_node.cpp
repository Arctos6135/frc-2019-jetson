#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <tables/ITableListener.h>

#include <string>
#include <memory>

#include <cstdlib>

// NetworkTables IP address
std::string nt_ip_addr;
// Subscriber for receiving vision result
ros::Subscriber result_sub;
ros::Subscriber angle_offset_sub;
ros::Subscriber x_offset_sub;
ros::Subscriber y_offset_sub;
// Service client for turning vision on or off
ros::ServiceClient vision_scli;
// The shared table for roboRIO/Jetson comms
std::shared_ptr<NetworkTable> table;
// Callback for when a result is generated
void result_callback(const std_msgs::Float64::ConstPtr &msg) {
	// Put the result into the table
	table->PutNumber("horizontal-angle", msg->data);
}
void angle_offset_callback(const std_msgs::Float64::ConstPtr &msg) {
    table->PutNumber("angle-offset", msg->data);
}
void x_offset_callback(const std_msgs::Float64::ConstPtr &msg) {
    table->PutNumber("x-offset", msg->data);
}
void y_offset_callback(const std_msgs::Float64::ConstPtr &msg) {
    table->PutNumber("y-offset", msg->data);
}

// Table listener
class VisionTableListener : public ITableListener {
	void ValueChanged(ITable *source, llvm::StringRef key, std::shared_ptr<nt::Value> value, bool is_new) override {
        // Check for vision enable entry
        // Make sure entry type is correct
		if(key.compare("vision-enable") == 0 && value->IsBoolean()) {
			std_srvs::SetBool args;
			args.request.data = value->GetBoolean();
			
			if(vision_scli.call(args)) {
				table->PutBoolean("enable-success", true);
			}
			else {
				table->PutBoolean("enable-success", false);

				ROS_FATAL_STREAM("Oh no! It's busted");
			}
		}
        // Jetson graceful shutdown
        else if(key.compare("shutdown") == 0 && value->IsBoolean() && value->GetBoolean()) {
            std::system("shutdown -P now");
        }
        else if(key.compare("restart-server") == 0 && value->isBoolean() && value->GetBoolean()) {
            table->PutBoolean("restart-server", false);
            std::system("rosnode kill /main_video_server");
        }
	}
};

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "nt_ros_translate_node");
	
	ros::NodeHandle node_handle("~");

	// Retrieve parameters
	node_handle.param<std::string>("nt_ip_addr", nt_ip_addr, "10.61.35.2");

	ROS_INFO_STREAM("Connecting to NetworkTables with IP " << nt_ip_addr);

	// NT setup
	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress(nt_ip_addr.c_str());
	NetworkTable::Initialize();

	table = NetworkTable::GetTable("roborio-jetson");

	// Add table listener
	VisionTableListener listener;
	table->AddTableListener(&listener);

	// Topic subscription
	result_sub = node_handle.subscribe("/vision_processing_node/result_horiz_angle", 1, result_callback);
    angle_offset_sub = node_handle.subscribe("/vision_processing_node/result_angle_offset", 1, angle_offset_callback);
    x_offset_sub = node_handle.subscribe("/vision_processing_node/result_x_offset", 1, x_offset_callback);
    y_offset_sub = node_handle.subscribe("/vision_processing_node/result_y_offset", 1, y_offset_callback);

	// Service client setup
	vision_scli = node_handle.serviceClient<std_srvs::SetBool>("/vision_processing_node/enable_vision");

	// Notify the rio that the Jetson vision is online
	table->PutBoolean("vision-online", true);

	ros::Rate rate(25);

	while(ros::ok()) {
		ros::spinOnce();

		rate.sleep();
	}
	
	// Notify the rio that vision has been shut down
	table->PutBoolean("vision-online", false);

	return 0;
}

