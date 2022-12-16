
#include <ros/ros.h>
#include <ros/rate.h>
//#include <rpwc/rpwc_gripper_cmd.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/robotEeState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>


trajectory_msgs::JointTrajectory send_hand_;
ros::Publisher pub_for_recording_, pub_hand_des_, pub_CommandHand_test_;
rpwc_msgs::RobotEeStateStamped lastCmdMsg_;

// bool callback_rpwc_gripper_single_cmd(rpwc::rpwc_gripper_cmd::Request  &req, rpwc::rpwc_gripper_cmd::Response &res)
// {
// 	send_hand_.points.clear();
//   	send_hand_.points.resize(1);
//   	send_hand_.points[0].positions.push_back(req.EE_cmd_.data);
//   	send_hand_.points[0].time_from_start = ros::Duration(0.3);
//   	pub_hand_des_.publish(send_hand_);

// 	return true;
// }

void callback_rpwc_gripper_cmd(const std_msgs::Float64::ConstPtr& msg)
{
	//HAND CLOSURE
  	send_hand_.points.clear();
  	send_hand_.points.resize(1);
  	send_hand_.points[0].positions.push_back(msg->data);
  	send_hand_.points[0].time_from_start = ros::Duration(0.3);
  	pub_hand_des_.publish(send_hand_);

	
	lastCmdMsg_.data.data = msg->data;
	lastCmdMsg_.header.stamp = ros::Time::now();
	pub_for_recording_.publish(lastCmdMsg_);

  	// std_msgs::Float64 hand_test;
  	// hand_test.data = msg->data;
  	//pub_CommandHand_test_.publish(hand_test);
}

bool callback_robot_curr_pose(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	res.data = lastCmdMsg_.data;
	res.header = lastCmdMsg_.header;
	return true;
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

	ros::init(argc, argv, "softhand_bridge_node");
	ros::NodeHandle nh;

	send_hand_.joint_names.resize(1);
  	send_hand_.joint_names[0] = "qbhand_synergy_joint";
	lastCmdMsg_.data.data = 0;

  	double rate_50Hz = 50.0;
	ros::Rate r_50HZ(rate_50Hz);

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
  	//Publisher
    pub_hand_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("qbhand/control/qbhand_synergy_trajectory_controller/command", 1);
    pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
	//Service Server
  	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", &callback_robot_curr_pose);
	
	while(ros::ok())
	{
		
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

