#ifndef JOINTOFFSET_H
#define JOINTOFFSET_H


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include "mar_robot_arm5e/ARM5Arm.h"
#include <mar_perception/VispUtils.h>
#include <tf/transform_listener.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <ar_pose/ARMarkers.h>

class JointOffset{

	ros::NodeHandle nh_;
	ARM5Arm *robot;
	vpColVector offset_;
	vpHomogeneousMatrix bMc, bMc_right, average_bMc;
	tf::StampedTransform cMm_tf;
	bool cMm_found, cMm_found_right;
	bool bMc_init, bMc_right_init;
	bool averaging_bMc; ///< Compute bMc averaging bMc and bMc_right
	ros::Subscriber joint_state_sub;
	ros::Subscriber marker_sub;
	ros::Publisher joint_state_pub;
	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;
	VispToTF v;
	

	void readJointsCallback(const sensor_msgs::JointState::ConstPtr& m);
	void markerCallback(const geometry_msgs::PoseStamped::ConstPtr& m);
	//void markerCallback(const ar_pose::ARMarkers::ConstPtr& m);
	vpHomogeneousMatrix markerToEndEffector(tf::Transform cMm_tf);

public:

	int reset_bMc();
	int reset_bMc( vpColVector initial_posture );
	/** @brief Compute bMc and bMc_right from a stereo pair cameras. */ 
	int reset_bMc_two_cams();
	/** @brief Compute bMc and bMc_right from a stereo pair cameras. Go to initial_posture first 
	 *  @param initial_posture: Starting position where the marker should be in field of view. */ 
	int reset_bMc_two_cams( vpColVector initial_posture );
	
	/** @brief Compute average bMc from bMc and bMc_right. */ 
	void compute_average_bMc();
	
	
	JointOffset(ros::NodeHandle& nh, std::string topic_joint_state, std::string topic_command_joint, std::string topic_joint_state_fixed, bool averaging_bMc = false );
	int get_bMc(vpHomogeneousMatrix &bMc_ret){
		if(bMc_init){
			bMc_ret=bMc;
			return 0;
		}
		return -1;
	}
	int get_bMc_right(vpHomogeneousMatrix &bMc_ret){
		if(bMc_init){
			bMc_ret=bMc_right;
			return 0;
		}
		return -1;
	}
	int get_average_bMc(vpHomogeneousMatrix &bMc_ret){
		if(bMc_init && averaging_bMc){
			bMc_ret=average_bMc;
			return 0;
		}
		return -1;
	}

};





#endif
