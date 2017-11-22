#include <butler/object_detector.h>

const std::string DEPTH_TOPIC_ = "/camera/depth/image";
const std::string RGB_TOPIC_ = "/camera/rgb/image_rect_color";

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Object Localization");
	ros::Rate r(100);
	
	ObjectDetector detector("detector");


	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}



ObjectDetector::ObjectDetector(const std::string& clientname_) : ObjDClient_(clientname_, 1) {
	depthS_ = n_.subscribe(DEPTH_TOPIC_, 0, &ObjectDetector::depthCallback, this);
	rgbS_ = n_.subscribe(RGB_TOPIC_, 0, &ObjectDetector::rgbCallback, this);
	cupPublisher_ = n_.advertise<darknet_ros_msgs::BoundingBoxes>("/cupbb", 0);

	ROS_INFO("waiting for action server to start");
	ObjDClient_.waitForServer();
	ROS_INFO("action server started");

	goal_.id = 0;
}

ObjectDetector::~ObjectDetector() {}

// new msg type? x,y,z probably
// publish local coordinates for cup
void ObjectDetector::publishCup() {
	return;
}

void ObjectDetector::depthCallback(const sensor_msgs::Image &img) {
	depthImg_ = img;
	return;
}

void ObjectDetector::rgbCallback(const sensor_msgs::Image &img) {
	rgbImg_ = img;
	return;
}
