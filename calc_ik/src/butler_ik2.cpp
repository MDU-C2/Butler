#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <calc_ik/Vector4.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/kdl.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#define PI 3.1415
int main(int argc, char **argv){
	ros::init(argc, argv, "tcp_to_jointspace");
	ros::NodeHandle node_handle;
	static const std::string PLANNING_GROUP = "arm";
	std::vector<double> joint_group_positions;
	ros::Rate r(50);
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	move_group.setPlanningTime(10);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	const robot_state::JointModelGroup *joint_model_group =
	move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	bool moved=false;
	char input;

	std::vector<double> zero_joints(5,0);
	std::vector<double> home_joints(5,0);
	home_joints[2]=-PI/2;	
	home_joints[3]=PI/2;

	geometry_msgs::PoseStamped robot_pose;
	geometry_msgs::PoseStamped current_pose;
	geometry_msgs::PoseStamped new_pose;
	tf::TransformListener tf_listener;
	tf::Quaternion qn;
	ros::Time now;
	double x,y,z,pitch=0,roll=0,amount=0.1;
	double hyp;
	while(ros::ok()){
		std::cout << "a: Enter pose values." << std::endl;
		std::cout << "s: Go to zero pose." << std::endl;
		std::cout << "d: Go to home pose." << std::endl;
		std::cout << "c: Show controls to move in cartesian." << std::endl;
		std::cout << "x: Exit." << std::endl;
		std::cin >> input;
		if (input=='a'){
			std::cout << "Enter x, y, z, pitch and roll for EEF." << std::endl;
			std::cout << "x:"; std::cin >> x; std::cout<<std::endl;
			std::cout << "y:"; std::cin >> y; std::cout<<std::endl;
			std::cout << "z:"; std::cin >> z; std::cout<<std::endl;
			std::cout << "pitch:"; std::cin >> pitch; std::cout<<std::endl;
			std::cout << "roll:"; std::cin >> roll; std::cout<<std::endl;
			robot_pose.header.frame_id="dummy_base";
			robot_pose.pose.position.x=x;
			robot_pose.pose.position.y=y;
			robot_pose.pose.position.z=z;
			hyp=sqrt(robot_pose.pose.position.x * robot_pose.pose.position.x + robot_pose.pose.position.y * robot_pose.pose.position.y);
			tfScalar rotz=acos(robot_pose.pose.position.x / hyp);
			if (robot_pose.pose.position.y<=0) rotz=rotz*-1;
			tfScalar roty=pitch;
			tfScalar rotx=roll;
			qn.setEulerZYX(rotz,roty,rotx);
			robot_pose.pose.orientation.x=(double)qn.x();
			robot_pose.pose.orientation.y=(double)qn.y();
			robot_pose.pose.orientation.z=(double)qn.z();
			robot_pose.pose.orientation.w=(double)qn.w();

			std::cout<< "The target pose is: \n"<<
			"x: " << robot_pose.pose.position.x << " \n" << 
			"y: " << robot_pose.pose.position.y << " \n" << 
			"z: " << robot_pose.pose.position.z << " \n" <<
			"qx: " << robot_pose.pose.orientation.x << " \n" << 
			"qy: " << robot_pose.pose.orientation.y << " \n" << 
			"qz: " << robot_pose.pose.orientation.z<< " \n" << 
			"qw: " << robot_pose.pose.orientation.w << std::endl;

			robot_pose.header.stamp = move_group.getRandomPose().header.stamp;
			now = ros::Time::now();
			tf_listener.waitForTransform("odom_gazebo","dummy_base", now, ros::Duration(3.0));
			tf_listener.transformPose("odom_gazebo",robot_pose,new_pose);

			std::cout<< "The new target pose is: \n"<<
			"x: " << new_pose.pose.position.x << " \n" << 
			"y: " << new_pose.pose.position.y << " \n" << 
			"z: " << new_pose.pose.position.z << " \n" <<
			"qx: " << new_pose.pose.orientation.x << " \n" << 
			"qy: " << new_pose.pose.orientation.y << " \n" << 
			"qz: " << new_pose.pose.orientation.z<< " \n" << 
			"qw: " << new_pose.pose.orientation.w << std::endl;
			move_group.setPoseTarget(new_pose);
			move_group.move();
		}else if(input=='s'){
			std::cout << "Going to zero position." << std::endl;
			move_group.setJointValueTarget(zero_joints);
			move_group.move();
		}else if(input=='d'){
			std::cout << "Going to home position." << std::endl;
			move_group.setJointValueTarget(home_joints);
			move_group.move();
		}else if(input=='x'){
			std::cout << "Exiting." << std::endl;
			return(0);
		}
		else if((input=='c') || (input=='v') || (input=='b')){
			switch(input){
				case 'c':
				std::cout << "Move with t&g in x, f&h in y, r&y in z. v&b to increase/decrease amount moved." << std::endl;
				break;
				case 'v':
				amount-=0.01;
				std::cout << "Increased by 0.01 to " << amount << "." << std::endl;
				break;
				case 'b':
				amount+=0.01;
				std::cout << "Increased by 0.01 to " << amount << "." << std::endl;
				break;
			}
		}
		else if((input=='f') || (input=='h') || (input=='g') || (input=='t') || (input=='r') || (input=='y') ){

			current_pose = move_group.getCurrentPose();
			now = ros::Time::now();
			tf_listener.waitForTransform("dummy_base","odom_gazebo", now, ros::Duration(3.0));
			tf_listener.transformPose("dummy_base",current_pose,robot_pose);

			switch(input){
				case 'f':
				std::cout << "Move 0.1 in y." << std::endl;
				robot_pose.pose.position.y += amount;
				break;
				case 'h':
				std::cout << "Move -0.1 in y." << std::endl;
				robot_pose.pose.position.y -= amount;
				break;
				case 't':
				std::cout << "Move 0.1 in x." << std::endl;
				robot_pose.pose.position.x += amount;
				break;
				case 'g':
				std::cout << "Move -0.1 in x." << std::endl;
				robot_pose.pose.position.x -= amount;
				break;
				case 'r':
				std::cout << "Move -0.1 in z." << std::endl;
				robot_pose.pose.position.z -= amount;
				break;
				case 'y':
				std::cout << "Move 0.1 in z." << std::endl;
				robot_pose.pose.position.z += amount;
				break;
			}
			robot_pose.header.frame_id="dummy_base";

			hyp=sqrt(robot_pose.pose.position.x * robot_pose.pose.position.x + robot_pose.pose.position.y * robot_pose.pose.position.y);
			tfScalar rotz=acos(robot_pose.pose.position.x / hyp);
			if (robot_pose.pose.position.y<=0) rotz=rotz*-1;
			tfScalar roty=pitch;
			tfScalar rotx=roll;
			qn.setEulerZYX(rotz,roty,rotx);
			robot_pose.pose.orientation.x=(double)qn.x();
			robot_pose.pose.orientation.y=(double)qn.y();
			robot_pose.pose.orientation.z=(double)qn.z();
			robot_pose.pose.orientation.w=(double)qn.w();

			std::cout<< "The target pose is: \n"<<
			"x: " << robot_pose.pose.position.x << " \n" << 
			"y: " << robot_pose.pose.position.y << " \n" << 
			"z: " << robot_pose.pose.position.z << " \n" <<
			"qx: " << robot_pose.pose.orientation.x << " \n" << 
			"qy: " << robot_pose.pose.orientation.y << " \n" << 
			"qz: " << robot_pose.pose.orientation.z<< " \n" << 
			"qw: " << robot_pose.pose.orientation.w << std::endl;

			robot_pose.header.stamp = move_group.getRandomPose().header.stamp;
			now = ros::Time::now();
			tf_listener.waitForTransform("odom_gazebo","dummy_base", now, ros::Duration(3.0));
			tf_listener.transformPose("odom_gazebo",robot_pose,new_pose);

			move_group.setPoseTarget(new_pose);
			move_group.move();
		}
	}
}