#ifndef _OBJECT_DETECTOR_H_
#define _OBJECT_DETECTOR_H_

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>



class ObjectDetector {
	public:
		ObjectDetector(const std::string&);
		~ObjectDetector();

		void publishCup();
		void depthCallback(const sensor_msgs::Image&);
		void rgbCallback(const sensor_msgs::Image&);
		void showDepth();
		void showRGB();
		void sendGoal(const int);

	private:
		ros::NodeHandle n_;
		ros::Subscriber depthS_, rgbS_;
		ros::Publisher cupPublisher_, rgbPub_, depthPub_;

		darknet_ros_msgs::CheckForObjectsGoal goal_;
		darknet_ros_msgs::CheckForObjectsResult result_;

		actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> ObjDClient_;

		sensor_msgs::Image rgbImg_, depthImg_;


};

#endif
