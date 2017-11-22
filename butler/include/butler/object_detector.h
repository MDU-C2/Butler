#ifndef _OBJECT_DETECTOR_H_
#define _OBJECT_DETECTOR_H_

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <sensor_msgs/Image.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>


extern const std::string DEPTH_TOPIC_;
extern const std::string RGB_TOPIC_;

class ObjectDetector {
	public:
		ObjectDetector(const std::string& clientname_);
		~ObjectDetector();

		void publishCup();
		void depthCallback(const sensor_msgs::Image& img);
		void rgbCallback(const sensor_msgs::Image& img);

	private:
		ros::NodeHandle n_;
		ros::Subscriber depthS_, rgbS_;
		ros::Publisher cupPublisher_;

		darknet_ros_msgs::CheckForObjectsGoal goal_;
		darknet_ros_msgs::CheckForObjectsResult result_;

		actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> ObjDClient_;

		sensor_msgs::Image rgbImg_, depthImg_;

};

#endif
