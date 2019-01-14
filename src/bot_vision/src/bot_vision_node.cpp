#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>

// Keeps track of whether vision processing is on
bool vision_on = false;
// Publisher for exposure control
ros::Publisher exposure_pub;
// Vision and normal exposure values
int vision_exposure, normal_exposure;
// Callback for the service
bool enable_vision_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp) {
	if(req.data) {
		vision_on = true;
		// Publish the exposure to set it
		std_msgs::Int32 m;
		m.data = vision_exposure;
		exposure_pub.publish(m);

		ROS_INFO_STREAM("Vision has been turned ON.");
	}
	else {
		vision_on = false;
		
		std_msgs::Int32 m;
		m.data = normal_exposure;
		exposure_pub.publish(m);
		
		ROS_INFO_STREAM("Vision has been turned OFF.");
	}
	
	resp.success = true;
	return true;
}

int main(int argc, char **argv) {
	// Init ROS
	ros::init(argc, argv, "bot_vision_node");
	// Get a reference to ths node
	ros::NodeHandle node_handle("~");

	// Init exposure publisher
	exposure_pub = node_handle.advertise<std_msgs::Int32>("/main_camera/exposure", 5);
	// Get the parameter values
	node_handle.param("vision_exposure", vision_exposure, 5);
	node_handle.param("normal_exposure", normal_exposure, 0);

	// Advertise the service
	ros::ServiceServer server = node_handle.advertiseService("enable_vision", enable_vision_callback);

	// Run forever until shutdown
	ros::spin();

	return 0;
}

