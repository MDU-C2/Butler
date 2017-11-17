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
//#include <boost/thread.hpp>
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

typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

//int getQ(const Vector3d &TCP, Vector4d &Q);

class ik{
public:
	ik();
	void tcpin(const geometry_msgs::Vector3& msg);
	int butlerIK(std::vector<double> &q_sol, geometry_msgs::Pose &target_pose);
	int getQ(const Vector3d &TCP, Vector4d &Q);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
	ros::NodeHandle node_handle_;
	double x,y,z;
	Vector3d tcp;
	ros::Subscriber tcp_node;
	ros::Publisher jointspace;
	Eigen::VectorXd q;
	Vector3d p0, p1, p2, p3, p4, p5;
	Vector3d z0, z1, z2, z3, z4;
	Eigen::MatrixXd hess;
	double linkOne,	linkTwo, height, t_off;
	double lambda;
};


ik::ik(){
	jointspace = node_handle_.advertise<calc_ik::Vector4>("jointspace", 0);
	tcp_node = node_handle_.subscribe("tcp", 0, &ik::tcpin, this);
	hess = Eigen::MatrixXd(6,6);
	q = Eigen::VectorXd(5,1);
//	Q << 0,0,0,0;
	tcp << 0,0,0;
	linkOne = 0.30;
	linkTwo = 0.20;
	height = 0.32;
	t_off = 0.05;
	lambda = 0.01;

	for(int k = 0; k < 6; ++k) hess(k, k) = lambda;
		q << 0.0, 0.0, 0.0, 0.0, 0.0;
	p0 << 0, 0, 0;
	p1 << 0, 0, 0;
	z0 << 0, 0, 1;
}

void ik::tcpin(const geometry_msgs::Vector3& msg)
{
	tcp << msg.x,msg.y,msg.z;
	std::cout << tcp;
	//getQ(tcp, Q);
	calc_ik::Vector4 vec;
	//vec.q0 = Q(0);
	//vec.q1 = Q(1);
	//vec.q2 = Q(2);
	//vec.q3 = Q(3);
	//jointspace.publish(vec);
}

#define PI 3.1415
int main(int argc, char **argv){
	ros::init(argc, argv, "tcp_to_jointspace");
	
	static const std::string PLANNING_GROUP = "arm";
	std::vector<double> joint_group_positions;
	ik balle = ik();
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
	/*double x,y,z;
	Vector3d tcp;
	Vector4d dQ;
	for (const auto &jg : joint_group_positions){
		std::cout << jg << " ";
	}
	std::cout << "nu är vi här vento" << std::endl;
	std::cout << std::endl;
	geometry_msgs::Pose target_pose;
	target_pose.position.x=0.3;target_pose.position.y=0.3;target_pose.position.z=0.5;
	std::cout << target_pose << std::endl;

	balle.butlerIK(joint_group_positions, target_pose);

	std::cout << "nu är vi här vento2" << std::endl;*/
	/*for (const auto &bg : joint_group_positions){
		std::cout << bg << " ";
	}*//*
move_group.setRandomTarget();
move_group.plan(my_plan);
move_group.move();*/

	/*Get the reference frame set by setPoseReferenceFrame(). By default this is the reference frame of the robot model. */
	std::string i;
	i=move_group.getPoseReferenceFrame();
	ROS_INFO("pose reference frame %s",i.c_str());
	i = move_group.getEndEffectorLink();
	ROS_INFO("eef %s",i.c_str());

	bool moved=false;

	char input;
/*
std::cout << move_group->getEndEffectorLink() << std::endl;*/
	/*
	for (double i=0.1; i<1.0; i=i+0.01){
		std::cout << i << std::endl;
		move_group.setPositionTarget(i,i,0.5,"hand");
		move_group.move();
	}*/
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
	double x,y,z,pitch,roll;
	double hyp;
	while(ros::ok()){
		std::cout << "a: Enter pose values." << std::endl;
		std::cout << "s: Go to zero pose." << std::endl;
		std::cout << "d: Go to home pose." << std::endl;
		std::cout << "x: Exit." << std::endl;
		std::cin >> input;
		if (input=='q'){
			move_group.setPoseReferenceFrame("dummy_base");
			robot_pose = move_group.getRandomPose();
			i=move_group.getPoseReferenceFrame();
			ROS_INFO("pose reference frame %s",i.c_str());

			std::cout<< "The target pose is: \n"<<
			"x: " << robot_pose.pose.position.x << " \n" << 
			"y: " << robot_pose.pose.position.y << " \n" << 
			"z: " << robot_pose.pose.position.z << " \n" <<
			"qx: " << robot_pose.pose.orientation.x << " \n" << 
			"qy: " << robot_pose.pose.orientation.y << " \n" << 
			"qz: " << robot_pose.pose.orientation.z<< " \n" << 
			"qw: " << robot_pose.pose.orientation.w << std::endl;

			move_group.setPoseTarget(robot_pose);
		}else if (input=='a'){
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
			ros::Time now = ros::Time::now();
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
		}else if(input=='s'){
			std::cout << "Going to zero position." << std::endl;
			move_group.setJointValueTarget(zero_joints);
		}else if(input=='d'){
			std::cout << "Going to home position." << std::endl;
			move_group.setJointValueTarget(home_joints);
		}else if(input=='x'){
			std::cout << "Exiting." << std::endl;
			return(0);
		}else if((input=='f') || (input=='h') || (input=='g') || (input=='t') || (input=='r') || (input=='y')){

			current_pose = move_group.getCurrentPose();
			ros::Time now = ros::Time::now();
			tf_listener.waitForTransform("dummy_base","odom_gazebo", now, ros::Duration(3.0));
			tf_listener.transformPose("dummy_base",current_pose,robot_pose);

			robot_pose = move_group.getCurrentPose();
			switch(input){
				case 'f':
				std::cout << "Move 0.1 in y." << std::endl;
				robot_pose.pose.position.y += 0.1;
				break;
				case 'h':
				std::cout << "Move -0.1 in y." << std::endl;
				robot_pose.pose.position.y -= 0.1;
				break;
				case 't':
				std::cout << "Move 0.1 in x." << std::endl;
				robot_pose.pose.position.x += 0.1;
				break;
				case 'g':
				std::cout << "Move 0.1 in x." << std::endl;
				robot_pose.pose.position.x -= 0.1;
				break;
				case 'r':
				std::cout << "Move -0.1 in z." << std::endl;
				robot_pose.pose.position.z -= 0.1;
				break;
				case 'y':
				std::cout << "Move 0.1 in z." << std::endl;
				robot_pose.pose.position.z += 0.1;
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
		}
		move_group.move();
	}
}


int ik::getQ(const Vector3d &TCP, Vector4d &Q)
{
/*	Eigen::VectorXd q(5);
	Vector3d p0, p1, p2, p3, p4, p5;
	Vector3d z0, z1, z2, z3, z4;*/
	//Vector3d tcp;

/*	double lambda = 0.01;
*//*	Eigen::MatrixXd hess(6,6);
*/
	/*for(int k = 0; k < 6; ++k) hess(k, k) = lambda;

	q << 0.5, 0.2, 0.5, 0.3, 0.3;*/
/*
	double linkOne = 0.30,
		linkTwo = 0.20,
		height = 0.32,
		t_off = 0.05;*/

	//tcp << TCP;
	q << PI/2, PI/4, PI/6, PI/6, PI/8;

	std::cout << TCP << std::endl;


	std::cout << "tcp:" << TCP << std::endl;
	std::cout << "q:" << q << std::endl;

	for(int k = 0; k < 100; ++k){
		z1 << std::sin(q(0)), -std::cos(q(0)), 0;
		z2 << std::sin(q(0)), -std::cos(q(0)), 0;
		z3 << std::sin(q(0)), -std::cos(q(0)), 0;
		z4 << std::sin(q(0)), -std::cos(q(0)), 0;

		p2 << 0, 0, height * std::sin(q(1));

		p3 << 0, 0, height * std::sin(q(1)) + height * std::sin(q(1) + q(2));

		p4 << std::cos(q(0)) * ((linkOne + t_off) * std::cos(q(1) + q(2) + q(3))),
		std::sin(q(0)) * ((linkOne + t_off) * std::cos(q(1) + q(2) + q(3))),
		height * std::sin(q(1)) + height * std::sin(q(1) + q(2)) + linkOne * std::sin(q(1) + q(2) + q(3));

		p5 << std::cos(q(0)) * ((linkOne + t_off) * std::cos(q(1) + q(2) + q(3)) + linkTwo * std::cos(q(1) + q(2) + q(3) + q(4))),
		std::sin(q(0)) * ((linkOne + t_off) * std::cos(q(1) + q(2) + q(3)) + linkTwo * std::cos(q(1) + q(2) + q(3) + q(4))),
		height * std::sin(q(1)) + height * std::sin(q(1) + q(2)) + linkOne * std::sin(q(1) + q(2) + q(3)) + linkTwo * std::sin(q(1) + q(2) + q(3) + q(4));
/*
		std::cout << "p5: " << p5 << std::endl;*/
		Eigen::MatrixXd Jac(6, 5);
		Jac << z0.cross(p5 - p0), z1.cross(p5 - p1), z2.cross(p5 - p2), z3.cross(p5 - p3), z4.cross(p5 - p4),
		z0, z1, z2, z3, z4;

		/*std::cout << Jac << std::endl;*/
		Eigen::VectorXd E(6);
		E << TCP - p5, 0, 0, 0;
		std::cout << "error: " << sqrt(E(0)*E(0) + E(1)*E(1) + E(2)*E(2)) << std::endl;


		if(sqrt(E(0)*E(0) + E(1)*E(1) + E(2)*E(2)) < 0.01) {
			//std::cout<<"error is low enough"<<std::endl;
			break;
		}
		Eigen::MatrixXd tempJ(6,6);
		tempJ = Jac * Jac.transpose() + hess;
		q += Jac.transpose() * tempJ.inverse() * E; 
/*		std::cout << "q:" << q << std::endl;
*/		/*std::cout << "inv: " << tempJ.determinant() << std::endl;*/
	}

	Q<< q(0), p3(2), q(3), q(4);

	return 0;
}

int ik::butlerIK(std::vector<double> &q_sol, geometry_msgs::Pose &target_pose ) //return 0 if solver was succesfull
{
	static bool firstrun=true;
	KDL::Frame target_kdl;
	static KDL::Chain drillChain;
	if(firstrun){ 
		firstrun=false;
		KDL::Tree my_tree;
		std::string robot_desc_string;
		node_handle_.param("robot_description", robot_desc_string, std::string());
		if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
			ROS_ERROR("Failed to construct kdl tree");
			return false;
		} 
		std::cout << "Get chain status:" << my_tree.getChain ( "base_link", "hand", drillChain) << std::endl;
	}
    KDL::JntArray q_solution(drillChain.getNrOfJoints()); //  init array for solution ti IK

    KDL::JntArray q_init(drillChain.getNrOfJoints()); //  tf::poseKDLToMsg(&pose);//Initialize guess of solution to IK

    std::cout << "joints: " << drillChain.getNrOfJoints() << std::endl;

    int jj = -1;
    for(const auto &qs : q_sol) q_init(++jj) = qs;

  //  std::cout << "snorre1" << std::endl;

    	static KDL::ChainIkSolverPos_LMA ikSolver(drillChain);
    tf::PoseMsgToKDL(target_pose,target_kdl);//Convert pose from geometry message to KDL
    int status = ikSolver.CartToJnt(q_init,target_kdl,q_solution);

  //  std::cout << "snorre2" << std::endl;

    for(int i=0;i<4;i++){ //copy the joint positions to the input/output parameter
       //out<<q_solution(i)<<endl;
    	q_sol[i] = q_solution(i);
    }
    return status; 
}