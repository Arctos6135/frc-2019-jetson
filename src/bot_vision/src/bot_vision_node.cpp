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
#define _USE_MATH_DEFINES
#include <cmath>

// If defined, the result will be logged to the info stream
#define _LOG_OUTPUT_
// If defined, a window will open with semi-processed images to aid with debugging
//#define _DEBUG_VISION_

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

float fullness_low = 0.75;
float fullness_high = 1.0;

int morph_kernel_size = 5;

int camera_horiz_fov = 53;
int camera_width = 640;
int camera_height = 480;

int camera_focal_len;

// The contour is checked with this function for validity
bool is_valid_contour(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
	// Check for fullness
	float contour_area = cv::contourArea(contour);
	float contour_fullness = contour_area / rect.size.area();

	if(contour_fullness <= fullness_high && contour_fullness >= fullness_low) {
		return true;
	}
	else {
		return false;
	}
}

double get_horiz_angle(const cv::Point2f &point) {
	double slope = (point.x - camera_width / 2) / camera_focal_len;
	return std::atan(slope) * 180 / M_PI;
}
// The image processing callback
void image_callback(const sensor_msgs::ImageConstPtr& msg) {
	if(vision_on) {
		cv::Mat original_image = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Mat hsv, mono;
		// Convert colour space
		cv::cvtColor(original_image, hsv, cv::COLOR_BGR2HSV_FULL);
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
        cv::findContours(mono, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Find bounding rects for each contour
        std::vector<cv::RotatedRect> rects;
        for(auto contour : contours) {
            // Check if contour is valid
			cv::RotatedRect bound = cv::minAreaRect(contour);
			
			if(is_valid_contour(contour, bound)) {
            	rects.push_back(bound);
			}
        }

        double x_angle = NAN;
        
        if(rects.size() >= 2) {
            // Find the two biggest contours
            int biggest = -1;
            int second_biggest = -1;
            for(int i = 0; i < rects.size(); i++) {
                if(biggest == -1 || rects[i].size.area() > rects[biggest].size.area()) {
                    second_biggest = biggest;
                    biggest = i;
                }
                else if(second_biggest == -1 || rects[i].size.area() > rects[second_biggest].size.area()) {
                    second_biggest = i;
                }
            }
			
			// Calculate the midpoint
            cv::Point2f mid1 = rects[biggest].center, mid2 = rects[second_biggest].center;
            cv::Point2f mid((mid1.x + mid2.x) / 2, (mid1.y + mid2.y) / 2);

			// Calculate the angle
			double angle = get_horiz_angle(mid);

			// Publish to the topic
			std_msgs::Float64 result;
			result.data = angle;
			result_pub.publish(result);

			#ifdef _LOG_OUTPUT_
			ROS_INFO("Target Angle: %f", angle);
			#endif
			
        }
		else {
			// Publish a NaN to indicate that nothing was found
			std_msgs::Float64 result;
			result.data = NAN;
			result_pub.publish(result);

			#ifdef _LOG_OUTPUT_
			ROS_INFO("Target not found.");
			#endif
		}
		
		#ifdef _DEBUG_VISION_
		cv::Mat copied;
		original_image.copyTo(copied);
		
		cv::drawContours(copied, contours, -1, cv::Scalar(0, 255, 255), 4);
		cv::imshow("Debug", copied);
		
		cv::waitKey(20);
		#endif
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
	node_handle.param("fullness_high", fullness_high, fullness_high);
	node_handle.param("fullness_low", fullness_low, fullness_low);
    node_handle.param("morph_kernel_size", morph_kernel_size, morph_kernel_size);
	node_handle.param("camera_width", camera_width, camera_width);
	node_handle.param("camera_height", camera_height, camera_height);
	node_handle.param("camera_horiz_fov", camera_horiz_fov, camera_horiz_fov);

	camera_focal_len = ((double) camera_width) / 2 / std::tan(((double) camera_horiz_fov) / 2 * M_PI / 180);

	// Reset camera back to normal exposure
	std_msgs::Int32 m;
	m.data = normal_exposure;
	exposure_pub.publish(m);

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

