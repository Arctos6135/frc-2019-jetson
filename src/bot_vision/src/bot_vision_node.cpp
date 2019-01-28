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
#include <utility>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <cmath>

// If defined, the result will be logged to the info stream
#define _LOG_OUTPUT_
// If defined, a window will open with semi-processed images to aid with debugging
//#define _DEBUG_VISION_

#define DEG(x) ((x) * 180 / M_PI)

extern bool vision_on;
// Publisher for final angle (result of processing)
ros::Publisher result_pub;
ros::Publisher angle_offset_pub;
ros::Publisher x_offset_pub;
ros::Publisher y_offset_pub;
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

int max_y_diff = 50;

int camera_horiz_fov = 61;
int camera_vert_fov = 37;
int camera_width = 1280;
int camera_height = 720;

int camera_horiz_f;
int camera_vert_f;

// Bounding box
double tape_width = 5.825572030188476;
double tape_gap = 8;

inline float combined_area(const std::pair<cv::RotatedRect, cv::RotatedRect> &contours) {
    return contours.first.size.area() + contours.second.size.area();
}

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
	double slope = (point.x - camera_width / 2) / camera_horiz_f;
	return std::atan(slope);
}
double get_vert_angle(const cv::Point2f &point) {
    double slope = (point.y - camera_width / 2) / camera_vert_f;
    return std::atan(slope);
}

double get_distance_v(const cv::Point2f &pt_high, const cv::Point2f &pt_low) {
    double theta = get_vert_angle(pt_low);
    double phi = get_vert_angle(pt_high);
    return tape_width / (std::tan(phi) - std::tan(theta));
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

        // Make a vector of pairs of rects
        // This will sore all the pairs of rects that are about the same height.
        std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> matching;
        // Go through all unique combinations
        for(int i = 0; i < rects.size(); i ++) {
            for(int j = i; j < rects.size(); j ++) {
                // Verify that the y diff is acceptable
                if(std::abs(rects[i].center.y - rects[j].center.y) <= max_y_diff) {
                    matching.push_back(std::make_pair(rects[i], rects[j]));
                }
            }
        }

        double x_angle = NAN;
        // Verify that there are one or more pairs of contours that match
        if(matching.size()) {
            // Find the two biggest pairs
            int biggest = -1;
            int second_biggest = -1;
            for(int i = 0; i < matching.size(); i++) {
                if(biggest == -1 || combined_area(matching[i]) > combined_area(matching[biggest])) {
                    second_biggest = biggest;
                    biggest = i;
                }
                else if(second_biggest == -1 || combined_area(matching[i]) > combined_area(matching[second_biggest])) {
                    second_biggest = i;
                }
            }
			
			// Calculate the midpoint
            cv::Point2f mid1 = matching[biggest].first.center, mid2 = matching[biggest].second.center;
            cv::Point2f mid((mid1.x + mid2.x) / 2, (mid1.y + mid2.y) / 2);

			// Calculate the angle
			double angle = get_horiz_angle(mid);

            // Calculate the distances from y coordinates of each piece of tape
            cv::Point2f points[4];
            matching[biggest].points(points);
            double min_y = std::min(points[0], std::min(points[1], std::min(points[2], points[3])));
            double max_y = std::max(points[0], std::max(points[1], std::max(points[2], points[3])));
            double dist1 = get_distance_v(max_y, min_y);

            matching[second_biggest].points(points);
            min_y = std::min(points[0], std::min(points[1], std::min(points[2], points[3])));
            max_y = std::max(points[0], std::max(points[1], std::max(points[2], points[3])));
            double dist2 = get_distance_v(max_y, min_y);
            // Calculate the angle difference
            double x_angle1 = get_horiz_angle(matching[biggest]);
            double x_angle2 = get_horiz_angle(matching[second_biggest]);
            double diff = std::max(x_angle1, x_angle2) - std::min(x_angle1, x_angle2);
            // Use the cosine law to find an estimate for the space between the two tapes
            double estimated_spacing = std::sqrt(dist1 * dist1 + dist2 * dist2 - 2 * dist1 * dist2 * std::cos(diff));
            // Divide the actual by the estimate to get an error multiplier
            double error = tape_gap / estimated_spacing;
            // Multiply both distances by the error to improve our estimate
            dist1 *= error;
            dist2 *= error;
            // Find out the relation of dist1 and dist2, as well as which angle is the one on the left
            double left_side, right_side;
            double left_angle, right_angle;
            if(x_angle1 < x_angle2) {
                left_side = dist1;
                right_side = dist2;
                left_angle = x_angle1;
                right_angle = x_angle2;
            }
            else {
                left_side = dist2;
                right_side = dist1;
                left_angle = x_angle1;
                right_angle = x_angle2;
            }
            // Calculate the angle offset of the line perpendicular to the target's plane to the centre of the camera
            double theta = std::acos((left_side * left_side + tape_gap * tape_gap - right_side * right_side) / (2 * left_side * tape_gap));
            double angle_offset = (M_PI / 2 - theta) + left_angle;
            // Calculate the coordinates of the centre of the target
            double x1 = std::cos(-left_angle + M_PI / 2) * left_side;
            double y1 = std::sin(-left_angle + M_PI / 2) * left_side;
            double x2 = std::cos(-right_angle + M_PI / 2) * right_side;
            double y2 = std::sin(-right_angle + M_PI / 2) * right_side;
            double target_x = (x1 + x2) / 2;
            double target_y = (y1 + y2) / 2;
            

			// Publish to the topic
            angle = DEG(angle);
			std_msgs::Float64 result;
			result.data = angle;
			result_pub.publish(result);

            result.data = DEG(angle_offset);
            angle_offset_pub.publish(result);

            result.data = target_x;
            x_offset_pub.publish(result);

            result.data = target_y;
            y_offset_pub.publish(result);

			#ifdef _LOG_OUTPUT_
			ROS_INFO("Target Angle: %f\nAngle Offset: %f\nX: %f, Y:%f", angle, angle_offset, target_x, target_y);

			#endif
        }
		else {
			// Publish a NaN to indicate that nothing was found
			std_msgs::Float64 result;
			result.data = NAN;
			result_pub.publish(result);
            angle_offset_pub.publish(result);
            x_offset_pub.publish(result);
            y_offset_pub.publish(result);

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
	result_pub = node_handle.advertise<std_msgs::Float64>("result_horiz_angle", 2);
    angle_offset_pub = node_handle.advertise<std_msgs::Float64>("result_angle_offset", 2);
    x_offset_pub = node_handle.advertise<std_msgs::Float64>("result_x_offset", 2);
    y_offset_pub = node_handle.advertise<std_msgs::Float64>("result_y_offset", 2);
	exposure_pub = node_handle.advertise<std_msgs::Int32>("/main_camera/exposure", 2);
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
    node_handle.param("max_y_diff", max_y_diff, max_y_diff);
	node_handle.param("camera_width", camera_width, camera_width);
	node_handle.param("camera_height", camera_height, camera_height);
	node_handle.param("camera_horiz_fov", camera_horiz_fov, camera_horiz_fov);
    node_handle.param("camera_vert_fov", camera_vert_fov, camera_vert_fov);

	camera_horiz_f = ((double) camera_width) / 2 / std::tan(((double) camera_horiz_fov) / 2 * M_PI / 180);
    camera_vert_f = ((double) camera_height) / 2 / std::tan(((double) camera_vert_fov) / 2 * M_PI / 180);

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

