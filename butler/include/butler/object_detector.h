#ifndef _OBJECT_DETECTOR_H_
#define _OBJECT_DETECTOR_H_

/*
 * ROS
 */
#include <ros/ros.h>
// image related
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// message related
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/image_encodings.h>
// action related
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

/*
 * C++
 */
#include <cmath>

typedef actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> Client_T_;

namespace ObjectID 
{
	const int Cup = 0;
}

/** @brief Main class for Object Detection
 *
 * 		Used to send images to darknet_ros and publish XYZ coordinates for detected objects
 * 		Cup position is published on the "/object_detector/cup_position" topic
 */
class ObjectDetector 
{
	public:
		/** @brief Default constructor for ObjectDetector class
		 *
		 * 		Initializes Action client, RGB, Depth, and CameraInfo subscribers, and Cup position publisher
		 */
		ObjectDetector(); 

		~ObjectDetector();

		/** @brief Callback function for depth image subscriber
		 *
		 *		stores the most recent depth image
		 */
		void depthCallback(const sensor_msgs::Image& img);

		/** @brief Callback function for rgb image subscriber
		 *
		 * 		stores the most recent RGB image
		 */
		void rgbCallback(const sensor_msgs::Image& img); 

		/** @brief Callback function for camera info subscriber
		 *
		 * 		retrieves the camera info
		 * 		specifically used to retrieve the camera matrices
		 */
		void cInfoCallback(const sensor_msgs::CameraInfo& info);

		/** @brief Main function to call the object detector
		 *
		 * 		latest rgb image is attached to goal message, as is the object id
		 */
		void sendGoal(const int id);

		/** @brief Detector callback function
		 *
		 * 		Used as a callback for sendGoal() and publishes the cup position
		 * 		Does K-Means clustering for each Depth image region of interest in order to extract a more precise depth measurement
		 */
		void detectorCallback(const actionlib::SimpleClientGoalState& state,
				const darknet_ros_msgs::CheckForObjectsResultConstPtr& result);

	private:
		/*
		 * Node
		 * Subscribers
		 * Publisher
		 */
		ros::NodeHandle n_;
		ros::Subscriber depthS_, rgbS_, cInfoS_;
		ros::Publisher cupPublisher_; 


		/*
		 * Action client for darknet_ros object detector
		 */
		Client_T_ client_;

		/*
		 * message variables
		 * cupPos_: x, y, z position published on Cup_Position_ topic
		 * rgbImg_, depthImg_: image messages from orbbec astra camera
		 * goal_: rgbImg_ and id for object detector
		 * result_: resulting bounding boxes and probabilities from detector
		 */  
		geometry_msgs::Point cupPos_;
		sensor_msgs::Image rgbImg_, depthImg_;
		sensor_msgs::CameraInfo cameraInfo_;
		darknet_ros_msgs::CheckForObjectsGoal goal_;
		darknet_ros_msgs::CheckForObjectsResult result_;

		/*
		 * retention of depth image, stored every time a goal is sent to darknet_ros for detection
		 * in order to match the depth to the correlated rgb image
		 */
		cv_bridge::CvImagePtr oldDepth_;

		const std::string Depth_Topic_		= "/camera/depth_registered/image_raw";
		const std::string RGB_Topic_ 		= "/usb_cam/image_raw";
		const std::string Action_Server_ 	= "/darknet_ros/check_for_objects";
		const std::string Camera_Info_		= "/camera/depth/camera_info";
		const std::string Cup_Position_		= "/object_detector/cup_position";

		/** @brief Mean function, returns the mean of three same-type values and is mean to look at
		 */
		template<typename T> inline T mean(const T x, const T y, const T z);

		/** @brief Draw function, draws bounding boxes and information labels on each detected object
		 *
		 * might as well have a separate function for it to avoid a mess
		 */
		void draw(cv_bridge::CvImagePtr& img_, const darknet_ros_msgs::BoundingBox& box_, int counter_);

		/** @brief Reshapes the boundingbox subset of img_ into a 1D array in obj_
		 */
		void reshape(cv::Mat& obj_, const cv_bridge::CvImagePtr& img_, const darknet_ros_msgs::BoundingBox& box_);

		/** @brief Maps depth image coordinates into meters
		 */
		void publishXYZ(const darknet_ros_msgs::BoundingBox& box_, double zd);
};

#endif
