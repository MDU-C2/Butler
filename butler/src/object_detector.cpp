#include <butler/object_detector.h>

const std::string Depth_Topic_		= "/camera/depth_registered/image_raw";
const std::string RGB_Topic_ 		= "/usb_cam/image_raw";
const std::string Action_Server_ 	= "/darknet_ros/check_for_objects";
const std::string Camera_Info_		= "/camera/depth/camera_info";
const std::string Cup_Position_		= "/object_detector/cup_position";

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
		detector.sendGoal(ObjectID::Cup);
	}

	return 0;
}

