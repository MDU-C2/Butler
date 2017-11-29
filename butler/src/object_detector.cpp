#include <butler/object_detector.h>

const std::string Depth_Topic_		= "/camera/depth_registered/image_raw";
const std::string Depth_Publisher_ 	= "/detector/depth";
const std::string RGB_Topic_ 		= "/usb_cam/image_raw";
const std::string RGB_Publisher_ 	= "/detector/rgb";
const std::string Action_Server_ 	= "/darknet_ros/check_for_objects";

/* 
 * TODO
 * extract distance to detected object
 * 3d coordinates???
 */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "objectdetector");
	ros::Time::init();
	ros::Rate r(0.1);

	ObjectDetector detector(Action_Server_);

	while(ros::ok()) 
	{
		r.sleep();
		ros::spinOnce();
		detector.sendGoal(0);
	}

	return 0;
}

