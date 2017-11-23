#include <butler/object_detector.h>

const std::string DEPTH_TOPIC_ = "/camera/depth_registered/image_raw";
const std::string RGB_TOPIC_ = "/usb_cam/image_raw";

int main(int argc, char** argv)
{
	ros::init(argc, argv, "objectdetector");
	ros::Time::init();
	ros::Rate r(100.0);

	ObjectDetector detector("CheckForObjects");

//	detector.publishCup();

	while(ros::ok()) {
		detector.showRGB();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}


ObjectDetector::ObjectDetector(const std::string& clientname_) : ObjDClient_(clientname_, 1) {
	depthS_ = n_.subscribe(DEPTH_TOPIC_, 0, &ObjectDetector::depthCallback, this);
	rgbS_ = n_.subscribe(RGB_TOPIC_, 0, &ObjectDetector::rgbCallback, this);
//	cupPublisher_ = n_.advertise<darknet_ros_msgs::BoundingBoxes>("/CupBB", 0);

//	ROS_INFO("waiting for action server to start");
//	ObjDClient_.waitForServer();
//	ROS_INFO("action server started");

	goal_.id = 0;
}

ObjectDetector::~ObjectDetector() {}

// new msg type? x,y,z probably
// publish local coordinates for cup
void ObjectDetector::publishCup() {
	return;
}

void ObjectDetector::depthCallback(const sensor_msgs::Image& img) {
	depthImg_ = img;
	return;
}

void ObjectDetector::rgbCallback(const sensor_msgs::Image& img) {
	rgbImg_ = img;
	return;
}

void ObjectDetector::showDepth() {
	cv_bridge::CvImagePtr dimg;
	try {
		dimg = cv_bridge::toCvCopy(depthImg_, sensor_msgs::image_encodings::TYPE_32FC1);
	} 
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("bridge error: %s", e.what());
		return;
	}
	cv::imshow("dwin", dimg->image);
	char wKey = cv::waitKey(1) && 0xFF;
	return;
}

void ObjectDetector::showRGB() {
	cv_bridge::CvImagePtr cimg;
	try {
		cimg = cv_bridge::toCvCopy(rgbImg_, sensor_msgs::image_encodings::RGB8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("bridge error: %s", e.what());
		return;
	}
	cv::imshow("cwin", cimg->image);
	char wKey = cv::waitKey(1) && 0xFF;
	return;
}

