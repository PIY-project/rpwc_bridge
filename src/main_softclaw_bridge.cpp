
#include <ros/ros.h>
#include <ros/rate.h>
#include <rpwc_msgs/robotEeCmd.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/robotEeState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>


trajectory_msgs::JointTrajectory send_hand_;
ros::Publisher pub_for_recording_, pub_hand_des_, pub_CommandHand_test_;
rpwc_msgs::RobotEeStateStamped lastCmdMsg_;

bool callback_rpwc_gripper_single_cmd(rpwc_msgs::robotEeCmd::Request  &req, rpwc_msgs::robotEeCmd::Response &res)
{
  	double position = req.state.position.data;
	if(req.state.position.data < 0.0)position = 0.0;
	else if (req.state.position.data > 1.0) position = 1.0;

	double velocity = 1;
	double force = 1;
	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();


	position = (position *1.5) - 0.5;
	send_hand_.points.clear();
  	send_hand_.points.resize(1);
  	send_hand_.points[0].positions.push_back(position);
  	send_hand_.points[0].time_from_start = ros::Duration(0.3);

	
	
	pub_hand_des_.publish(send_hand_);

	return true;
}

void callback_rpwc_gripper_cmd(const rpwc_msgs::RobotEeStateStamped::ConstPtr& msg)
{
	double position = msg->position.data;
	if(msg->position.data < 0.0)position = 0.0;
	else if (msg->position.data > 1.0) position = 1.0;

	double velocity = 1;
	double force = 1;

	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();

	position = (position *1.5) - 0.5;
	send_hand_.points.clear();
  	send_hand_.points.resize(1);
  	send_hand_.points[0].positions.push_back(position);
  	send_hand_.points[0].time_from_start = ros::Duration(0.3);

	
	

	pub_hand_des_.publish(send_hand_);
	pub_for_recording_.publish(lastCmdMsg_);
}

bool callback_robot_curr_pose(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	lastCmdMsg_.header.stamp = ros::Time::now();
	res.state = lastCmdMsg_;
	return true;
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

	ros::init(argc, argv, "softclaw_bridge_node");
	ros::NodeHandle nh;

	send_hand_.joint_names.resize(1);
  	send_hand_.joint_names[0] = "qbsoftclaw_deflection_virtual_joint";
	lastCmdMsg_.position.data = 0;
	lastCmdMsg_.velocity.data = 0.125;
	lastCmdMsg_.force.data = 0.625;
	lastCmdMsg_.header.stamp = ros::Time::now();

  	double rate_10Hz = 10.0;
	ros::Rate r_10HZ(rate_10Hz);

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
  	//Publisher
    pub_hand_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("qbsoftclaw/control/qbsoftclaw_deflection_trajectory_controller/command", 1);
    pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
	//Service Server
	ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("rpwc_EE_cmd", &callback_rpwc_gripper_single_cmd);
  	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", &callback_robot_curr_pose);
	
	while(ros::ok())
	{
		
		ros::spinOnce();
		r_10HZ.sleep();
	}// end while()
	return 0;
}

