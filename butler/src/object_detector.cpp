#include <butler/object_detector.h>

static const std::string DEPTH_TOPIC_ = "/camera/depth_registered/image_raw";
static const std::string RGB_TOPIC_ = "/usb_cam/image_raw";
static const std::string RGB_PUB_ = "/detector/rgb";
static const std::string DEPTH_PUB_ = "/detector/depth";
static const std::string GOAL_PUB_ = "/darknet_ros/check_for_objects";


/* 
 * TODO
 * get action results
 * the rest
 */

int main(int argc, char** argv)
{
	ros::init(argc, argv, "objectdetector");
	ros::Time::init();
	ros::Rate r(0.05);

	ObjectDetector detector(GOAL_PUB_);

	while(ros::ok()) {
		r.sleep();
		ros::spinOnce();
		detector.sendGoal(0);
	}
	return 0;
}

ObjectDetector::ObjectDetector(const std::string& clientname) : ObjDClient_(clientname, true) {
	depthS_ = n_.subscribe(DEPTH_TOPIC_, 0, &ObjectDetector::depthCallback, this);
	rgbS_ = n_.subscribe(RGB_TOPIC_, 0, &ObjectDetector::rgbCallback, this);

	ROS_INFO("waiting for action server to start");
	ObjDClient_.waitForServer();
}

ObjectDetector::~ObjectDetector() {}

// new msg type? x,y,z probably
// publish local coordinates for cup???
void ObjectDetector::publishCup() {
	return;
}

void ObjectDetector::sendGoal(const int id) {
	goal_.id = id;
	goal_.image = rgbImg_;
	ObjDClient_.sendGoal(goal_);
	ROS_INFO("sent goal");
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

