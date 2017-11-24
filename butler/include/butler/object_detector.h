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


extern const std::string Depth_Topic_;
extern const std::string RGB_Topic_;
extern const std::string RGB_Publisher_;
extern const std::string Depth_Publisher_;
extern const std::string Action_Server_; 

typedef actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> Client_T_;

class ObjectDetector 
{
	private:
		ros::NodeHandle n_;
		ros::Subscriber depthS_, rgbS_;
		ros::Publisher cupPublisher_, rgbPub_, depthPub_;

		darknet_ros_msgs::CheckForObjectsGoal goal_;
		darknet_ros_msgs::CheckForObjectsResult result_;

		Client_T_ Client_;

		sensor_msgs::Image rgbImg_, depthImg_;
		cv_bridge::CvImagePtr oldDepth_;

	public:
		ObjectDetector(const std::string& clientname) 
			: Client_(clientname, true) 
		{
			depthS_ = n_.subscribe(Depth_Topic_, 0, &ObjectDetector::depthCallback, this);
			rgbS_ = n_.subscribe(RGB_Topic_, 0, &ObjectDetector::rgbCallback, this);

			ROS_INFO("waiting for action server to start");
			Client_.waitForServer();
		}

		~ObjectDetector() 
		{
		}

		void publishCup() 
		{
			return;
		}

		void depthCallback(const sensor_msgs::Image& img) 
		{
			depthImg_ = img;
			return;

		}

		void rgbCallback(const sensor_msgs::Image& img) 
		{
			rgbImg_ = img;
			return;
		}

		void showDepth()
		{
			cv_bridge::CvImagePtr dimg;
			try 
			{
				dimg = cv_bridge::toCvCopy(depthImg_, sensor_msgs::image_encodings::TYPE_32FC1);
			} 
			catch (cv_bridge::Exception& e) 
			{
				ROS_ERROR("cv_bridge error: %s", e.what());
				return;
			}
			cv::imshow("dwin", dimg->image);
			char wKey = cv::waitKey(1) && 0xFF;
			return;
		}

		void showRGB()
		{
			cv_bridge::CvImagePtr cimg;
			try 
			{
				cimg = cv_bridge::toCvCopy(rgbImg_, sensor_msgs::image_encodings::RGB8);
			}
			catch (cv_bridge::Exception& e) 
			{
				ROS_ERROR("cv_bridge error: %s", e.what());
				return;
			}
			cv::imshow("cwin", cimg->image);
			char wKey = cv::waitKey(1) && 0xFF;
			return;
		}

		void sendGoal(const int id) 
		{
			goal_.id = id;
			goal_.image = rgbImg_;

			try
			{
				oldDepth_ = cv_bridge::toCvCopy(depthImg_, sensor_msgs::image_encodings::TYPE_32FC1);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge error: %s", e.what());
				return;
			}

			Client_.sendGoal(goal_,
					boost::bind(&ObjectDetector::detectorCallback, this, _1, _2),
					Client_T_::SimpleActiveCallback(),
					Client_T_::SimpleFeedbackCallback());
			//			Client_.sendGoal(goal_);
			ROS_INFO("sent goal");
			return;
		}

		void detectorCallback(const actionlib::SimpleClientGoalState& state,
				const darknet_ros_msgs::CheckForObjectsResultConstPtr& result) 
		{
			short k = 0;
			for(const auto& bb : result->boundingBoxes.boundingBoxes) 
			{
				ROS_INFO("#%d: %s; Prob: %3f", ++k, bb.Class.c_str(), bb.probability);
				cv::line(oldDepth_->image, cv::Point(bb.xmin, bb.ymin), cv::Point(bb.xmax, bb.ymin), 1.0);
				cv::line(oldDepth_->image, cv::Point(bb.xmin, bb.ymin), cv::Point(bb.xmin, bb.ymax), 1.0);
				cv::line(oldDepth_->image, cv::Point(bb.xmin, bb.ymax), cv::Point(bb.xmax, bb.ymax), 1.0);
				cv::line(oldDepth_->image, cv::Point(bb.xmax, bb.ymin), cv::Point(bb.xmax, bb.ymax), 1.0);
			}
			cv::imshow("win", oldDepth_->image);
			char wKey = cv::waitKey(1) & 0xFF;
			return;
		}
};

#endif
