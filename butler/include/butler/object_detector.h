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
	public:
		ObjectDetector(const std::string& clientname) 
			: client_(clientname, true) 
		{
			depthS_ = n_.subscribe(Depth_Topic_, 0, &ObjectDetector::depthCallback, this);
			rgbS_ = n_.subscribe(RGB_Topic_, 0, &ObjectDetector::rgbCallback, this);

			ROS_INFO("waiting for action server to start");
			client_.waitForServer();
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

			client_.sendGoal(goal_,
					boost::bind(&ObjectDetector::detectorCallback, this, _1, _2),
					Client_T_::SimpleActiveCallback(),
					Client_T_::SimpleFeedbackCallback());
			ROS_INFO("sent goal");

			//client_.waitForResult(ros::Duration(5.0));
			return;
		}

		void detectorCallback(const actionlib::SimpleClientGoalState& state,
				const darknet_ros_msgs::CheckForObjectsResultConstPtr& result) 
		{
			short m = 0;
			for(const auto& bb : result->boundingBoxes.boundingBoxes) 
			{
				cv::Mat obj((bb.xmax - bb.xmin) * (bb.ymax - bb.ymin), 1, CV_32F);
				for(int k = 0; k < (bb.xmax - bb.xmin); ++k)
				{
					for(int l = 0; l < (bb.ymax - bb.ymin); ++l)
					{
						float cc = (float) oldDepth_->image.at<unsigned char>(l + bb.ymin, k + bb.xmin) / 255.0;
						obj.at<float>(l + (k * (bb.ymax - bb.ymin)), 0) = cc;
					}
				}

				cv::Mat labels, centers;
				/*
				 * K-Means clustering to find a decent depth to the detected object
				 */
				cv::kmeans(obj, 2, labels, 
						cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 1.0), 
						3, cv::KMEANS_PP_CENTERS, centers);

				float dist = centers.at<float>(0, 0);
				float avg = 0.0;
				int count = 0;
				for(int k = 0; k < (bb.xmax - bb.xmin); ++k)
				{
					for(int l = 0; l < (bb.ymax - bb.ymin); ++l)
					{
						avg += thresh(oldDepth_->image.at<unsigned char>(l + bb.ymin, k + bb.xmin), count, 0.1);
						int idx = labels.at<int>(l + (k * (bb.ymax - bb.ymin)), 0);
						oldDepth_->image.at<unsigned char>(l + bb.ymin, k + bb.xmin) = (unsigned char) (centers.at<float>(idx, 0) * 255);
						if(dist < centers.at<float>(idx, 0)) dist = centers.at<float>(idx, 0);
					}
				}
				avg /= (float) count;

				std::ostringstream s;
				s << std::setprecision(3) << m << "; P: " << bb.probability;

				cv::putText(oldDepth_->image, s.str(), cv::Point(bb.xmin, bb.ymin), cv::FONT_HERSHEY_PLAIN, 1.0, 1.0);
				cv::line(oldDepth_->image, cv::Point(bb.xmin, bb.ymin), cv::Point(bb.xmin, bb.ymax), 1.0, 2);
				cv::line(oldDepth_->image, cv::Point(bb.xmin, bb.ymax), cv::Point(bb.xmax, bb.ymax), 1.0, 2);
				cv::line(oldDepth_->image, cv::Point(bb.xmax, bb.ymin), cv::Point(bb.xmax, bb.ymax), 1.0, 2);

				ROS_INFO("#%d: %s; Prb: %3f, K: %3f, A: %3f", ++m, bb.Class.c_str(), bb.probability, dist, avg);
			}
			cv::imshow("depthwin", oldDepth_->image);
			char wKey = cv::waitKey(1) & 0xFF;
			return;
		}

	private:
		ros::NodeHandle n_;
		ros::Subscriber depthS_, rgbS_;
		ros::Publisher cupPublisher_, rgbPub_, depthPub_;

		darknet_ros_msgs::CheckForObjectsGoal goal_;
		darknet_ros_msgs::CheckForObjectsResult result_;

		Client_T_ client_;

		sensor_msgs::Image rgbImg_, depthImg_;
		cv_bridge::CvImagePtr oldDepth_;

		inline float thresh(const unsigned char& d, int& c, const float v)
		{
			if((float) (d / 255.0) > v) 
			{
				c++;
				return (float) (d / 255.0);
			}
			else return 0.0;
		}

};

#endif
