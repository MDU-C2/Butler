#include <butler/object_detector.h>

const std::string DEPTH_TOPIC_ = "/camera/depth_registered/image_raw";
const std::string RGB_TOPIC_ = "/usb_cam/image_raw";
const std::string RGB_PUB_ = "/detector/rgb";
const std::string DEPTH_PUB_ = "/detector/depth";
const std::string GOAL_PUB_ = "/darknet_ros/check_for_objects";


/* 
 * TODO
 * get action results
 * the rest
 */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "objectdetector");
	ros::Time::init();
	ros::Rate r(0.1);

	ObjectDetector detector(GOAL_PUB_);

	while(ros::ok()) {
		r.sleep();
		ros::spinOnce();
		detector.sendGoal(0);
	}
	return 0;
}

