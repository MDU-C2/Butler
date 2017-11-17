#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <calc_ik/Vector4.h>
#include <sensor_msgs/Image.h>

#define PI 3.1415
void callbackf(const sensor_msgs::Image& depth_img){
	std::cout << "Received an image message." << std::endl;

	int i=0; int j=0;
	for (i=0;i<depth_img.step*depth_img.height;i++)
	{
		for (int i = 0; i < depth_img.width; ++i)
		{
			std::cout << (unsigned)depth_img.data[i] << ", ";
		}
		std::cout << std::endl << std::endl;
		
	}
	std::cout << depth_img.step << 
	" is step and height: " << depth_img.height << 
	" and  width: " << depth_img.width << std::endl;
	ros::Duration(20).sleep();
}
int main(int argc, char **argv){
	ros::init(argc, argv, "listen_for_image");
	ros::NodeHandle node_handle;



	const std::string topic="/kinect/depth_registered/image_raw";

	/*boost::shared_ptr<sensor_msgs::Image const> sharedPtr;
	sensor_msgs::Image depth_img;
	sharedPtr = ros::topic::waitForMessage<sensor_msgs::Image>(topic, ros::Duration(10));
	depth_img = *sharedPtr;*/


	ros::Subscriber image_in;
	image_in = node_handle.subscribe(topic, 1, &callbackf);

/*

	std::cout << depth_img.step << 
	" is step and height: " << depth_img.height << 
	" and  width: " << depth_img.width << std::endl;
	int i=0; int j=0;
	for (i=0;i<depth_img.step;i++){
		for (j=0;j<depth_img.height;j++){
			std::cout << (uint)depth_img.data[i,j];
		}
		std::cout << std::endl;
	}*/




	while (ros::ok()){ros::spin();}
}