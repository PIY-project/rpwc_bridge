#include <summit_xls_steel_cl_bridge.h>

summit_xls_steel_cl_bridge::summit_xls_steel_cl_bridge()
{
	//Subscriber
	odom_sub_ = n_.subscribe("/robot/robotnik_base_control/odom", 1, &summit_xls_steel_cl_bridge::odom_callback, this);
	sub_rpwc_control_poses_ = n_.subscribe("rpwc_pose_des", 1, &summit_xls_steel_cl_bridge::callback_rpwc_control_poses, this);
	
	//Publisher
    pub_poses_control_ = n_.advertise<rpwc_msgs::RobotMobileBaseControl>("poses_control", 1);
    pub_curr_pos_ = n_.advertise<geometry_msgs::PoseStamped>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &summit_xls_steel_cl_bridge::callback_robot_curr_pose, this);
	current_time, old_time = ros::Time::now();
}

summit_xls_steel_cl_bridge::~summit_xls_steel_cl_bridge()
{

}

void summit_xls_steel_cl_bridge::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose_curr.pose = msg->pose.pose;
  pose_curr.header = msg->header;
  pose_curr.header.stamp = ros::Time::now();
  pub_curr_pos_.publish(pose_curr);
}

// void summit_xls_steel_cl_bridge::callback_curr_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
// 	pose_curr = *msg;
//  	pose_curr.header.stamp = ros::Time::now();
//  	pub_curr_pos_.publish(pose_curr);
	
// }

void summit_xls_steel_cl_bridge::callback_rpwc_control_poses(const rpwc_msgs::RobotMobileBaseControl::ConstPtr& msg)
{
	pub_poses_control_.publish(msg);
}

bool summit_xls_steel_cl_bridge::callback_robot_curr_pose(rpwc_msgs::robotMobileBaseState::Request  &req, rpwc_msgs::robotMobileBaseState::Response &res)
{
 	geometry_msgs::PoseStamped robot_curr_pose;
	robot_curr_pose = pose_curr;

	robot_curr_pose.header.stamp = ros::Time::now();

	res.pose = robot_curr_pose; 
	return true;
}

