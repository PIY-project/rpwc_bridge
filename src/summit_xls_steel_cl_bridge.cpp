#include <summit_xls_steel_cl_bridge.h>

summit_xls_steel_cl_bridge::summit_xls_steel_cl_bridge()
{
	//Subscriber
	sub_curr_pos_ = n_.subscribe("robot/robotnik_base_control/odom", 1, &summit_xls_steel_cl_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &summit_xls_steel_cl_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::Pose>("pose_des", 1);
    pub_curr_pos_ = n_.advertise<nav_msgs::Odometry>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &summit_xls_steel_cl_bridge::callback_robot_curr_pose, this);
  	
}

summit_xls_steel_cl_bridge::~summit_xls_steel_cl_bridge()
{

}


void summit_xls_steel_cl_bridge::callback_curr_pose(const nav_msgs::Odometry::ConstPtr& msg)
{

	odom_curr = *msg;
	odom_curr.header.stamp = ros::Time::now();
	pub_curr_pos_.publish(odom_curr);
}

void summit_xls_steel_cl_bridge::callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg)
{
	geometry_msgs::Pose send_pose;
	send_pose.position.x = msg->position.x;
	send_pose.position.y = msg->position.y;
	send_pose.position.z = msg->position.z;
	send_pose.orientation.w = msg->orientation.w;
  	send_pose.orientation.x = msg->orientation.x;
  	send_pose.orientation.y = msg->orientation.y;
  	send_pose.orientation.z = msg->orientation.z;
	pub_pos_des_.publish(send_pose);
}

bool summit_xls_steel_cl_bridge::callback_robot_curr_pose(rpwc_msgs::robotMobileBaseState::Request  &req, rpwc_msgs::robotMobileBaseState::Response &res)
{
	nav_msgs::Odometry robot_curr_pose;
	robot_curr_pose = odom_curr;

	robot_curr_pose.header.stamp = ros::Time::now();

	res.odometry = robot_curr_pose;
	return true;
}


