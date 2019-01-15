#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <cmath>

extern bool vision_on;
// Publisher for final angle (result of processing)
ros::Publisher result_pub;
// Vision processing parameters
int thresh_high_h = 130;
int thresh_high_s = 255;
int thresh_high_v = 255;
int thresh_low_h = 80;
int thresh_low_s = 80;
int thresh_low_v = 70;

int morph_kernel_size = 5;

int camera_horiz_fov = 53;
int camera_width = 640;
int camera_width = 480;

// The image processing callback
void image_callback(const sensor_msgs::ImageConstPtr& msg) {
	if(vision_on) {
		cv::Mat hsv, mono;
		// Convert colour space
		cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, hsv, cv::COLOR_BGR2HSV_FULL);
		// Threshold
		cv::inRange(hsv, cv::Scalar(thresh_low_h, thresh_low_s, thresh_low_v),
				cv::Scalar(thresh_high_h, thresh_high_s, thresh_high_v), mono);
		// Open and close
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_size, morph_kernel_size),
				cv::Point(morph_kernel_size / 2, morph_kernel_size / 2));
		cv::morphologyEx(mono, mono, cv::MORPH_OPEN, kernel);
		cv::morphologyEx(mono, mono, cv::MORPH_CLOSE, kernel);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        // Return as list and simplify to only end points
        cv::findContours(mono, contours, cv::CV_RETR_LIST, cv::CV_CHAIN_APPROX_SIMPLE);

        // Find bounding rects for each contour
        std::vector<cv::RotatedRect> rects;
        for(auto contour : contours) {
            // Add checks here if necessary
            rects.push_back(cv::minAreaRect(contour));
        }

        double x_angle = NAN;
        
        if(rects.size() >= 2) {
            // Find the two biggest contours
            int biggest = -1;
            int second_biggest = -1;
            for(int i = 0; i < rects.size(); rects++) {
                if(biggest == -1 || rects[i].size.area() > rects[biggest].size.area()) {
                    second_biggest = biggest;
                    biggest = i;
                }
                else if(second_biggest == -1 || rects[i].size.area() > rects[second_biggest].size.area()) {
                    second_biggest = i;
                }
            }

            cv::Point2f mid1 = rects[biggest].center, mid2 = rects[second_biggest].center;
            cv::Point2f mid((mid1.x + mid2.x) / 2, (mid1.y + mid2.y) / 2);

        }

		//cv::imshow("view", mono);
		//cv::waitKey(30);
	}
}

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

	// Init publishers
	result_pub = node_handle.advertise<std_msgs::Float64>("result_horiz_angle", 5);
	exposure_pub = node_handle.advertise<std_msgs::Int32>("/main_camera/exposure", 5);
	// Get the parameter values
	node_handle.param("vision_exposure", vision_exposure, 5);
	node_handle.param("normal_exposure", normal_exposure, 0);
    node_handle.param("thresh_high_h", thresh_high_h, thresh_high_h);
    node_handle.param("thresh_high_s", thresh_high_s, thresh_high_s);
    node_handle.param("thresh_high_v", thresh_high_v, thresh_high_v);
    node_handle.param("thresh_low_h", thresh_low_h, thresh_low_h);
    node_handle.param("thresh_low_s", thresh_low_s, thresh_low_s);
    node_handle.param("thresh_low_v", thresh_low_v, thresh_low_v);
    node_handle.param("morph_kernel_size", morph_kernel_size, morph_kernel_size);

	// Set up image transport stuff
	image_transport::ImageTransport im_transport(node_handle);
	// Subscribe to the raw image topic
	// Use a queue size of 1 so unprocessed images are discarded
	image_transport::Subscriber im_sub = im_transport.subscribe("/main_camera/image_raw", 1, image_callback);

	// Advertise the service
	ros::ServiceServer server = node_handle.advertiseService("enable_vision", enable_vision_callback);

	// Run forever until shutdown
	ros::spin();

	return 0;
}

