#include <ros/ros.h>
#include <ros/rate.h>
#include <rpwc_msgs/robotEeCmd.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/robotEeState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <iostream>


trajectory_msgs::JointTrajectory cmd_msg_;
ros::Publisher pub_for_recording_, pub_gripper_des_;
rpwc_msgs::RobotEeStateStamped lastCmdMsg_;
double gripperFeedbackState_ = 0.0;

bool inizialized_ = false;

bool callback_server_rpwc_gripper_cmd(rpwc_msgs::robotEeCmd::Request  &req, rpwc_msgs::robotEeCmd::Response &res)
{
  	double position = req.state.position.data;
	if(req.state.position.data < 0.0)position = 0.0;
	else if (req.state.position.data > 1.0) position = 1.0;

	double velocity = 1;
	double force = 1;

	cmd_msg_.joint_names.resize(1);
  	cmd_msg_.joint_names[0] = "qbhand_synergy_joint";
	cmd_msg_.points.clear();
  	cmd_msg_.points.resize(1);
  	cmd_msg_.points[0].positions.push_back(position);
  	cmd_msg_.points[0].time_from_start = ros::Duration(0.3);
	pub_gripper_des_.publish(cmd_msg_);

	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();

	return true;
}


void callback_rpwc_gripper_cmd(const rpwc_msgs::RobotEeStateStamped::ConstPtr& msg)
{
	double position = msg->position.data;
	if(msg->position.data < 0.0)position = 0.0;
	else if (msg->position.data > 1.0) position = 1.0;

	double velocity = 1;
	double force = 1;

	/*double velocity = msg->velocity.data;
	if(msg->velocity.data < 0.125)velocity = 0.125;
	else if (msg->velocity.data > 1.0) velocity = 1.0;

	double force = msg->force.data;
	if(msg->force.data < 0.625)force = 0.625;
	else if (msg->force.data > 1.0) force = 1.0;*/

	cmd_msg_.joint_names.resize(1);
  	cmd_msg_.joint_names[0] = "qbhand_synergy_joint";
	cmd_msg_.points.clear();
  	cmd_msg_.points.resize(1);
  	cmd_msg_.points[0].positions.push_back(position);
  	cmd_msg_.points[0].time_from_start = ros::Duration(0.3);

	if((lastCmdMsg_.position.data != position) || (lastCmdMsg_.velocity.data != velocity) || (lastCmdMsg_.force.data != force) ) pub_gripper_des_.publish(cmd_msg_);
	
	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();
}

bool callback_robot_curr_pose(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	lastCmdMsg_.header.stamp = ros::Time::now();
	res.state = lastCmdMsg_;
	return true;
}

void callback_gripper_feedback(const sensor_msgs::JointState::ConstPtr& msg)
{
	gripperFeedbackState_ = msg->position[0];
	if(!inizialized_)
	{
		inizialized_ = true;
		lastCmdMsg_.position.data = gripperFeedbackState_;
		if(lastCmdMsg_.position.data > 0.80) lastCmdMsg_.position.data = 1.0;
		lastCmdMsg_.velocity.data = 0;
		lastCmdMsg_.force.data = 0;
	}
}

bool callback_robot_curr_state(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	res.state.position.data = gripperFeedbackState_;
	return true;
}

void pub_for_recording_callback()
{
	ros::NodeHandle nhThread;
	double rate_10Hz = 10.0;
	ros::Rate r_10HZ(rate_10Hz);
	
	while(ros::ok())
	{
		pub_for_recording_.publish(lastCmdMsg_);
		ros::spinOnce();
		r_10HZ.sleep();
	}// end while()

	return;
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "softhand_bridge_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
    spinner.start();

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
	ros::Subscriber sub_gripper_feedback = nh.subscribe("qbhand/control/joint_states", 1, &callback_gripper_feedback);
  	//Publisher
    pub_gripper_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("qbhand/control/qbhand_synergy_trajectory_controller/command", 1);
    pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
	//Service Server
	ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("rpwc_EE_cmd", &callback_server_rpwc_gripper_cmd);
  	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", &callback_robot_curr_pose);
	ros::ServiceServer server_robot_curr_state = nh.advertiseService("rpwc_robot_curr_state", &callback_robot_curr_state);


	// fare un thread con pub for recording a 10 hz
	std::thread pubForRecordingThread(&pub_for_recording_callback);
	pubForRecordingThread.detach();

	ros::waitForShutdown();
	return 0;
}

