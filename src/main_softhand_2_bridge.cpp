
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
	double velocity = req.state.velocity.data;

	if(req.state.position.data < 0.0)position = 0.0;
	else if (req.state.position.data > 1.0) position = 1.0;

	if(req.state.velocity.data < 0.0)velocity = 0.0;
	else if (req.state.velocity.data > 1.0) velocity = 1.0;

	
	double force = 1;

	send_hand_.points.clear();
  	send_hand_.points.resize(1);
  	send_hand_.points[0].positions.push_back(position);
  	send_hand_.points[0].positions.push_back(velocity);
  	send_hand_.points[0].time_from_start = ros::Duration(0.3);

	
	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();

	pub_hand_des_.publish(send_hand_);

	return true;
}

void callback_rpwc_gripper_cmd(const rpwc_msgs::RobotEeStateStamped::ConstPtr& msg)
{
	double position = msg->position.data;
	double velocity = msg->velocity.data;

	if(msg->position.data < 0.0)position = 0.0;
	else if (msg->position.data > 1.0) position = 1.0;

	if(msg->velocity.data < 0.0)velocity = 0.0;
	else if (msg->velocity.data > 1.0) velocity = 1.0;

	
	double force = 1;


	send_hand_.points.clear();
  	send_hand_.points.resize(1);
  	send_hand_.points[0].positions.push_back(position);
	send_hand_.points[0].positions.push_back(velocity);
  	send_hand_.points[0].time_from_start = ros::Duration(0.3);

	
	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();

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

	ros::init(argc, argv, "softhand_2_bridge_node");
	ros::NodeHandle nh;

	send_hand_.joint_names.resize(2);
  	send_hand_.joint_names[0] = "qbhand2m_manipulation_joint";
  	send_hand_.joint_names[1] = "qbhand2m_synergy_joint";
	lastCmdMsg_.position.data = 0.0;
	lastCmdMsg_.velocity.data = 0.0;
	lastCmdMsg_.force.data = 0.0;
	lastCmdMsg_.header.stamp = ros::Time::now();

  	double rate_10Hz = 10.0;
	ros::Rate r_10HZ(rate_10Hz);

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
  	//Publisher
    pub_hand_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("qbhand2m/control/qbhand2m_synergies_trajectory_controller/command", 1);
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





// #include <ros/ros.h>
// #include <ros/rate.h>
// #include <rpwc/rpwc_gripper_cmd.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <trajectory_msgs/JointTrajectory.h>


// trajectory_msgs::JointTrajectory send_hand_;
// ros::Publisher pub_hand_des_;

// // bool callback_rpwc_gripper_single_cmd(rpwc::rpwc_gripper_cmd::Request  &req, rpwc::rpwc_gripper_cmd::Response &res)
// // {
// // 	send_hand_.points.clear();
// //   	send_hand_.points.resize(1);
// //   	send_hand_.points[0].positions.push_back(req.EE_cmd_.data);
// //   	send_hand_.points[0].time_from_start = ros::Duration(0.3);
// //   	pub_hand_des_.publish(send_hand_);

// // 	return true;
// // }

// // void callback_rpwc_gripper_cmd(const std_msgs::Float64MultiArray::ConstPtr& msg)
// // {
// // 	//HAND CLOSURE
// //   	send_hand_.points.clear();
// //   	send_hand_.points.resize(1);
// //   	send_hand_.points[0].positions.push_back(msg->data[0]);
// //   	send_hand_.points[0].positions.push_back(msg->data[1]);
// //   	send_hand_.points[0].time_from_start = ros::Duration(0.3);
// //   	// send_hand_.points[1].positions.push_back(msg->data[1]);
// //   	// send_hand_.points[1].time_from_start = ros::Duration(0.3);
// //   	pub_hand_des_.publish(send_hand_);

// // }

// void callback_rpwc_gripper_cmd(const std_msgs::Float64::ConstPtr& msg)
// {
// 	//HAND CLOSURE
//   	send_hand_.points.clear();
//   	send_hand_.points.resize(1);
//   	send_hand_.points[0].positions.push_back(0.0);
//   	send_hand_.points[0].positions.push_back(msg->data);
//   	send_hand_.points[0].time_from_start = ros::Duration(0.3);
//   	// send_hand_.points[1].positions.push_back(msg->data[1]);
//   	// send_hand_.points[1].time_from_start = ros::Duration(0.3);
//   	pub_hand_des_.publish(send_hand_);

// }


// //-----------------------------------------------------
// //                                                 main
// //-----------------------------------------------------
// int main(int argc, char **argv)
// {

// 	ros::init(argc, argv, "softhand_2_bridge_node");
// 	ros::NodeHandle nh;

// 	// std::string topic_qbhand_command, qbhand_name_joint;
// 	// nh.getParam("topic_qbhand_command", topic_qbhand_command);
// 	// nh.getParam("qbhand_name_joint", qbhand_name_joint);

// 	// std::cout<<"qbhand_name_joint"<< qbhand_name_joint<<std::endl;

// 	send_hand_.joint_names.resize(2);
//   	send_hand_.joint_names[0] = "qbhand2m_manipulation_joint";
//   	send_hand_.joint_names[1] = "qbhand2m_synergy_joint";

//   	double rate_50Hz = 50.0;
// 	ros::Rate r_50HZ(rate_50Hz);

// 	//Subscriber
// 	// ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_2cmd", 1, &callback_rpwc_gripper_cmd);
// 	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
//   	//Publisher
//     pub_hand_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("/qbhand2m/control/qbhand2m_synergies_trajectory_controller/command", 1);

// 	// Service Server
//   	// ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("rpwc_EE_single_cmd", &callback_rpwc_gripper_single_cmd);



// 	while(ros::ok())
// 	{
		
// 		ros::spinOnce();
// 		r_50HZ.sleep();
// 	}// end while()
// 	return 0;
// }

