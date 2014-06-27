
#include <ros/ros.h>
/*#include <actionlib/server/simple_action_server.h>
#include <grasper/plugHotStabAction.h>
#include <grasper/unplugHotStabAction.h>*/

#include <mar_robot_arm5e/ARM5Arm.h>
#include <grasper/joint_offset.h>
#include <mar_ros_bridge/mar_params.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <tf/tfMessage.h>
#include <mar_perception/VirtualImage.h>
#include <std_srvs/Empty.h>

/*
ros::NodeHandle nh_;

	vpColVector initial_posture_;

	std::string joint_state_, joint_state_command_, joint_state_fixed_;
	JointOffset* joint_offset_;
	ARM5Arm* robot_;

	void init(std::string name){
	

		nh_.getParam("joint_state", joint_state_);
		nh_.getParam("joint_state_command", joint_state_command_);
		nh_.getParam("joint_state_fixed", joint_state_fixed_);

		initial_posture_=mar_params::paramToVispColVector(&nh_, "initial_posture");



		joint_offset_=new JointOffset(nh_, joint_state_, joint_state_command_, joint_state_fixed_);
		robot_=new ARM5Arm(nh_, joint_state_fixed_, joint_state_command_);

}*/

int main(int argc, char** argv){
	ros::init(argc, argv, "move");
	ros::NodeHandle nh;
	
	
	ARM5Arm* robot=new ARM5Arm(nh, "/arm5e/joint_state_angle", "/arm5e/command_angle");

	vpColVector initial_posture(5);
	
	initial_posture[0]=atof(argv[1]);	
	initial_posture[1]=atof(argv[2]);	
	initial_posture[2]=atof(argv[3]);	
	initial_posture[3]=atof(argv[4]);	
	initial_posture[4]=atof(argv[5]);
	
	ROS_INFO_STREAM("Going to: " << initial_posture);
	vpColVector current_joints;
	robot->getJointValues(current_joints);
	if(initial_posture.size()!=5)
		initial_posture=current_joints;
	while((initial_posture-current_joints).euclideanNorm()>0.02 && ros::ok()){
		robot->setJointVelocity(initial_posture-current_joints);
		ros::spinOnce();
		robot->getJointValues(current_joints);
		std::cout<<(initial_posture-current_joints).euclideanNorm()<<std::endl;
	}
	
	ROS_INFO("ARRIVED");
    

	return 0;
	}
