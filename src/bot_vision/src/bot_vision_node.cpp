#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "bot_vision_node");


	return 0;
}

