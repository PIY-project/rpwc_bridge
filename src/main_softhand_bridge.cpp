
#include <ros/ros.h>
#include <ros/rate.h>
//#include <rpwc/rpwc_gripper_cmd.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/robotEeState.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>


trajectory_msgs::JointTrajectory send_hand_;
ros::Publisher pub_for_recording_, pub_hand_des_, pub_CommandHand_test_;
rpwc_msgs::RobotEeStateStamped lastCmdMsg_;

void callback_rpwc_gripper_cmd(const rpwc_msgs::RobotEeStateStamped::ConstPtr& msg)
{
	double position = msg->position.data;
	if(msg->position.data < 0.0)position = 0.0;
	else if (msg->position.data > 1.0) position = 1.0;

	
	/*double velocity = msg->velocity.data;
	if(msg->velocity.data < 0.125)velocity = 0.125;
	else if (msg->velocity.data > 1.0) velocity = 1.0;

	double force = msg->force.data;
	if(msg->force.data < 0.625)force = 0.625;
	else if (msg->force.data > 1.0) force = 1.0;*/

	send_hand_.points.clear();
  	send_hand_.points.resize(1);
  	send_hand_.points[0].positions.push_back(position);
  	send_hand_.points[0].time_from_start = ros::Duration(0.3);

	
	

	pub_hand_des_.publish(send_hand_);
	//pub_for_recording_.publish(lastCmdMsg_);
}

void callback_cmd(const std_msgs::Float64::ConstPtr& msg)
{
	// if(msg->points.size() > 0 && msg->points[0].positions.size() > 0)
	// {
	// 	double velocity = 1;
	// 	double force = 1;
	// 	lastCmdMsg_.position.data = msg->points[0].positions[0];
	// 	lastCmdMsg_.velocity.data = velocity;
	// 	lastCmdMsg_.force.data = force;
	// 	lastCmdMsg_.header.stamp = ros::Time::now();
		
	// 	pub_for_recording_.publish(lastCmdMsg_);
	// }
	double velocity = 1;
	double force = 1;
	lastCmdMsg_.position.data = msg->data;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();
	
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

	ros::init(argc, argv, "softhand_bridge_node");
	ros::NodeHandle nh;

	send_hand_.joint_names.resize(1);
  	send_hand_.joint_names[0] = "qbhand_synergy_joint";
	lastCmdMsg_.position.data = 0;
	lastCmdMsg_.velocity.data = 0.125;
	lastCmdMsg_.force.data = 0.625;
	lastCmdMsg_.header.stamp = ros::Time::now();

  	double rate_100Hz = 100.0;
	ros::Rate r_100HZ(rate_100Hz);

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
	// ros::Subscriber sub_cmd = nh.subscribe("control/qbhand_synergy_trajectory_controller/command", 1, &callback_cmd);
	ros::Subscriber sub_cmd = nh.subscribe("hand_closure", 1, &callback_cmd);
  	//Publisher
    pub_hand_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("control/qbhand_synergy_trajectory_controller/command", 1);
    pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
	//Service Server
  	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", &callback_robot_curr_pose);
	
	while(ros::ok())
	{
		
		ros::spinOnce();
		r_100HZ.sleep();
	}// end while()
	return 0;
}

