
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



	vpColVector initial_posture_;

	std::string joint_state_, joint_state_command_, joint_state_fixed_;
	JointOffset* joint_offset_;
	ARM5Arm* robot_;

	void init(std::string name, ros::NodeHandle nh_ ){
	

		nh_.getParam("joint_state", joint_state_);
		nh_.getParam("joint_state_command", joint_state_command_);
		nh_.getParam("joint_state_fixed", joint_state_fixed_);

		initial_posture_=mar_params::paramToVispColVector(&nh_, "initial_posture");



		joint_offset_=new JointOffset(nh_, joint_state_, joint_state_command_, joint_state_fixed_);
		robot_=new ARM5Arm(nh_, joint_state_fixed_, joint_state_command_);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "get_bMc");
	//HotStab hotStab(ros::this_node::getName());
	
    ros::NodeHandle nh_;
    init(ros::this_node::getName(), nh_);
    ROS_INFO("Initializing Joint Offset (bMc)");    
    joint_offset_->reset_bMc(initial_posture_);
    vpHomogeneousMatrix bMc;
    if(joint_offset_->get_bMc(bMc)==0)
		ROS_INFO_STREAM("bMc is: " << std::endl << bMc);
	else
		ROS_INFO("bMC not found");
	ros::spin();
	return 0;
	
	/* First bMc found for reference only
	 * 0.03905909885  0.7491046278  0.6612992087  -0.2960783322  
-0.9991522374  0.03789416151  0.01608847269  -0.06422862159  
-0.01300742967  -0.6613669853  0.7499496767  0.3065145907  
0  0  0  1
*/
	
	
	
	}
