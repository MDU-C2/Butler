#ifndef _OBJECT_DETECTOR_H_
#define _OBJECT_DETECTOR_H_

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


extern const std::string Depth_Topic_;
extern const std::string RGB_Topic_;
extern const std::string Action_Server_; 
extern const std::string Cup_Position_;


typedef actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> Client_T_;

class ObjectDetector 
{
	public:
		ObjectDetector(const std::string& clientname) 
			: client_(clientname, true) 
		{
			depthS_ = n_.subscribe(Depth_Topic_, 0, &ObjectDetector::depthCallback, this);
			rgbS_ = n_.subscribe(RGB_Topic_, 0, &ObjectDetector::rgbCallback, this);
			cupPublisher_ = n_.advertise<geometry_msgs::Point>(Cup_Position_, 1);

			ROS_INFO("waiting for action server to start");
			client_.waitForServer();
			ROS_INFO("connected to action server");
		}

		~ObjectDetector() 
		{
		}

		/*
		 * callback function for depth image subscriber
		 */
		void depthCallback(const sensor_msgs::Image& img) 
		{
			depthImg_ = img;
			return;

		}

		/*
		 * callback function for rgb image subscriber
		 */
		void rgbCallback(const sensor_msgs::Image& img) 
		{
			rgbImg_ = img;
			return;
		}

		/*
		 * unnecessary function to display depth image
		 * should be removed eventually
		 */
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

		/*
		 * unnecessary function to display rgb image
		 * should be removed eventually
		 */
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

		/*
		 * function to send goal to detector
		 * latest rgb image is attached to the message
		 * as is the object id in question
		 */
		void sendGoal(const int id) 
		{
			goal_.id = id;
			goal_.image = rgbImg_;

			/*
			 * convert sensor_msgs::Image to opencv image
			 */
			try
			{
				oldDepth_ = cv_bridge::toCvCopy(depthImg_, sensor_msgs::image_encodings::TYPE_32FC1);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge error: %s", e.what());
				return;
			}

			/*
			 * send goal function
			 * binds callback function
			 */
			client_.sendGoal(goal_,
					boost::bind(&ObjectDetector::detectorCallback, this, _1, _2),
					Client_T_::SimpleActiveCallback(),
					Client_T_::SimpleFeedbackCallback());
			ROS_INFO("sent goal");

			return;
		}

		/*
		 * detector callback function
		 * does the fun stuff such as extract and publish cup coordinates
		 */
		void detectorCallback(const actionlib::SimpleClientGoalState& state,
				const darknet_ros_msgs::CheckForObjectsResultConstPtr& result) 
		{
			short m = 0;
			for(const auto& bb : result->boundingBoxes.boundingBoxes) 
			{
				/*
				 * reshape bounding box pixels into a 1D array for k-means
				 */
				cv::Mat obj((bb.xmax - bb.xmin) * (bb.ymax - bb.ymin), 1, CV_32F);
				for(int k = 0; k < (bb.xmax - bb.xmin); ++k)
				{
					for(int l = 0; l < (bb.ymax - bb.ymin); ++l)
					{
						float cc = (float) oldDepth_->image.at<unsigned char>(l + bb.ymin, k + bb.xmin) / 255.0;
						obj.at<float>(l + (k * (bb.ymax - bb.ymin)), 0) = cc;
					}
				}

				/*
				 * K-Means clustering to find a decent depth estimate to the detected object
				 */
				cv::Mat labels, centers;
				cv::kmeans(obj, 3, labels, 
						cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 50, 0.1), 
						3, cv::KMEANS_PP_CENTERS, centers);

				float dist = 0.0;	//k-means distance

				/*
				 * depth image is imprecise
				 * 3 clusters increases assurance that the cup map is extracted correctly
				 * background/foreground/misreadings generally give darker/brigher values than the cup
				 */
				dist = mean(centers.at<float>(0), centers.at<float>(1), centers.at<float>(2));

				/*
				 * draw bounding boxes and info on depth image
				 * for debugging purposes, will be removed in the end i guess
				 */
				draw(oldDepth_, bb, m);

				/*
				 * publisher and info output
				 */
				cupPos_.x = 0.0;
				cupPos_.y = 0.0;
				cupPos_.z = (double) dist; //depth
				cupPublisher_.publish(cupPos_);
				ROS_INFO("#%d: %s; Prb: %3f, K: %3f", ++m, bb.Class.c_str(), bb.probability, dist);
			}
			/*
			 * displays depth image with bounding boxes
			 * for debugging purposes, will be removed in the end i guess
			 */
			cv::imshow("depthwin", oldDepth_->image);
			char wKey = cv::waitKey(1) & 0xFF;
			return;
		}

	private:
		/*
		 * Node
		 * Subscribers
		 * Publisher
		 */
		ros::NodeHandle n_;
		ros::Subscriber depthS_, rgbS_;
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
		darknet_ros_msgs::CheckForObjectsGoal goal_;
		darknet_ros_msgs::CheckForObjectsResult result_;

		/*
		 * retention of depth image, stored every time a goal is sent to darknet_ros for detection
		 */
		cv_bridge::CvImagePtr oldDepth_;

		/*
		 * threshholding function, also keeps track of "successful" addition with int& c
		 */
		inline float thresh(const unsigned char value_, int& counter_, const float threshval_)
		{
			if((float) (value_ / 255.0) > threshval_) 
			{
				counter_++;
				return (float) (value_ / 255.0);
			}
			else return 0.0;
		}

		/*
		 * mean function, returns the mean of three same-type values and is mean to look at
		 */
		template<typename T>
			inline T mean(T x, T y, T z)
			{
				if(((x < y) && (x > z)) || ((x < z) && (x > y))) return x;
				if(((y < x) && (y > z)) || ((y < z) && (y > x))) return y;
				if(((z < x) && (z > y)) || ((z < y) && (z > x))) return z;
			}

		/*
		 * draw function
		 * might as well have a separate function for it to avoid a mess
		 */
		inline void draw(cv_bridge::CvImagePtr& img_, const darknet_ros_msgs::BoundingBox& box_, const int counter_)
		{
			std::ostringstream str_;
			str_ << std::setprecision(3) << m << "; P: " << box_.probability;
			cv::putText(img_->image, str_.str(), cv::Point(box_.xmin, box_.ymin), cv::FONT_HERSHEY_PLAIN, 1.0, 1.0);
			cv::line(img_->image, cv::Point(box_.xmin, box_.ymin), cv::Point(box_.xmin, box_.ymax), 1.0, 2);
			cv::line(img_->image, cv::Point(box_.xmin, box_.ymax), cv::Point(box_.xmax, box_.ymax), 1.0, 2);
			cv::line(img_->image, cv::Point(box_.xmax, box_.ymin), cv::Point(box_.xmax, box_.ymax), 1.0, 2);
			return;
		}
};

#endif
