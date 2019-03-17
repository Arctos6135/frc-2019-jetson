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
int thresh_low_v = 80;

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
double tape_height = 5.825572030188476;
double tape_gap = 11.0629666927;

double get_distance_v(double, double);
inline double get_rect_distance(const cv::RotatedRect &rect) {
	cv::Point2f points[4];
	rect.points(points);
	double min_y = std::min(points[0].y, std::min(points[1].y, std::min(points[2].y, points[3].y)));
    double max_y = std::max(points[0].y, std::max(points[1].y, std::max(points[2].y, points[3].y)));
	return get_distance_v(max_y, min_y);
}

inline float rank(const std::pair<cv::RotatedRect, cv::RotatedRect> &rects) {
    return get_rect_distance(rects.first) + get_rect_distance(rects.second);
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
float rotatedrect_angle(const cv::RotatedRect &rect) {
	if(rect.size.width < rect.size.height) {
		return rect.angle + 180;
	}
	else {
		return rect.angle + 90;
	}
}

inline bool operator==(const cv::RotatedRect &a, const cv::RotatedRect &b) {
	return a.center.x == b.center.x && a.center.y == b.center.y && a.size.width == b.size.width 
			&& a.size.height == b.size.height && a.angle == b.angle;
}
inline bool operator!=(const cv::RotatedRect &a, const cv::RotatedRect &b) {
	return !(a == b);
}

bool is_valid_pair(const std::pair<cv::RotatedRect, cv::RotatedRect> &rects, const std::vector<cv::RotatedRect> &all_rects) {
	const cv::RotatedRect *left, *right;
	if (rects.first.center.x < rects.second.center.x) {
		left = &rects.first;
		right = &rects.second;
	}
	else {
		right = &rects.first;
		left = &rects.second;
	}

	if(rotatedrect_angle(*left) < 90 && rotatedrect_angle(*right) > 90) {
		for(const auto &rect : all_rects) {
			if(rect != *left && rect != *right) {
				if(rect.center.x > left->center.x && rect.center.y < right->center.y) {
					return false;
				}
			}
		}
		return true;
	}
	return false;
}

inline double get_horiz_angle(const double x) {
    double slope = (x - camera_width / 2) / camera_horiz_f;
    return std::atan(slope);
}
inline double get_horiz_angle(const cv::Point2f &point) {
	return get_horiz_angle(point.x);
}
inline double get_vert_angle(const double y) {
    double slope = (y - camera_height / 2) / camera_vert_f;
    return std::atan(slope);
}
inline double get_vert_angle(const cv::Point2f &point) {
    return get_vert_angle(point.y);
}

inline double get_distance_v(const double high, const double low) {
    double theta = get_vert_angle(low);
    double phi = get_vert_angle(high);
    return tape_height / (std::tan(phi) - std::tan(theta));
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

bool publish_processed = false;
bool publish_processed_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp) {
	publish_processed = req.data;
	resp.success = true;
	return true;
}

image_transport::Publisher processed_pub;

inline void publish_image(const cv::Mat &img) {
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	processed_pub.publish(msg);
}

void draw_rotatedrect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color = cv::Scalar(0, 0, 255), int thickness = 3) {
	cv::Point2f points[4];
	rect.points(points);

	cv::line(img, points[0], points[1], color, thickness);
	cv::line(img, points[1], points[2], color, thickness);
	cv::line(img, points[2], points[3], color, thickness);
	cv::line(img, points[3], points[0], color, thickness);
}
void draw_combined_rect(cv::Mat &img, const std::pair<cv::RotatedRect, cv::RotatedRect> &rects, const cv::Scalar &color = cv::Scalar(0, 0, 255), int thickness = 3) {
	cv::Point2f points[8];
	rects.first.points(points);
	rects.second.points(points + 4); // I love pointers
	auto rect = cv::boundingRect(points);
	cv::rectangle(img, rect.tl(), rect.br(), color, thickness);
}

// The image processing callback
void image_callback(const sensor_msgs::ImageConstPtr& msg) {
	if(vision_on) {
		// Get a modifiable copy
		// Since the camera images are either rgb8 or yuyv a copy has to be made anyways
		cv::Mat original_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
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

		if(publish_processed) {
			publish_image(mono);
		}

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
        // This will store all the pairs of rects that are about the same height.
        std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> matching;
        // Go through all unique combinations
        for(int i = 0; i < rects.size(); i ++) {
            for(int j = i + 1; j < rects.size(); j ++) {
                // Verify that the y diff is acceptable
                if(std::abs(rects[i].center.y - rects[j].center.y) <= max_y_diff) {
					// Verify other things
					auto rect_pair = std::make_pair(rects[i], rects[j]);
					if(is_valid_pair(rect_pair, rects)) {
						matching.push_back(rect_pair);
					}
                }
            }
        }

        double x_angle = NAN;
        // Verify that there are one or more pairs of contours that match
        if(matching.size()) {
            // Find the best pair
            int best = -1;
            for(int i = 0; i < matching.size(); i++) {
				// Note: the rank function was changed to give a low number for high ranks.
                if(best == -1 || rank(matching[i]) < rank(matching[best])) {
                    best = i;
                }
            }

			if(publish_image) {

				for(int i = 0; i < matching.size(); i ++) {
					
				}
			}
			
			// Calculate the midpoint
            auto &tapes = matching[best];
            cv::Point2f mid1 = tapes.first.center, mid2 = tapes.second.center;
            cv::Point2f mid((mid1.x + mid2.x) / 2, (mid1.y + mid2.y) / 2);

			// Calculate the angle
			double angle = get_horiz_angle(mid);

            // Calculate the distances from y coordinates of each piece of tape
            double dist1 = get_rect_distance(tapes.first);
            double dist2 = get_rect_distance(tapes.second);


            // Calculate the angle difference
            double x_angle1 = get_horiz_angle(tapes.first.center);
            double x_angle2 = get_horiz_angle(tapes.second.center);
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
                left_angle = x_angle2;
                right_angle = x_angle1;
            }


			#ifdef _LOG_OUTPUT_
			ROS_INFO("Left: %f, Right: %f", left_side, right_side);
			ROS_INFO("Left Angle: %f, Right Angle:%f", left_angle, right_angle);
			#endif

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

			angle_offset = DEG(angle_offset);
            result.data = angle_offset;
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

	// Init publisher for processed images
	image_transport::ImageTransport it(node_handle);
	processed_pub = it.advertise("processed_image", 1);

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
	ros::ServiceServer enable_vision_server = node_handle.advertiseService("enable_vision", enable_vision_callback);
	ros::ServiceServer publish_processed_server = node_handle.advertiseService("publish_processed", publish_processed_callback);

	// Run forever until shutdown
	ros::spin();

	return 0;
}

