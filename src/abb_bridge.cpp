#include <abb_bridge.h>

abb_bridge::abb_bridge()
{
	//Subscriber
	sub_curr_pos_ = n_.subscribe("curr_pose", 1, &abb_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &abb_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::Pose>("pose_des", 1);
    pub_curr_pos_ = n_.advertise<geometry_msgs::PoseStamped>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &abb_bridge::callback_robot_curr_pose, this);
  	
  	server_switch_controller_ = n_.advertiseService("rpwc_switch_arm_controller", &abb_bridge::callback_switch_controller, this);
  	// client_switch_controller_ = n_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
  	client_switch_controller_ = n_.serviceClient<abb_driver::srv_abb_controller>("abb_controller");

	T_base_2_EE_ = Eigen::Matrix4d::Identity();

	first_quat_base_EE_ = true;
}

abb_bridge::~abb_bridge()
{

}

void abb_bridge::callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
	msg_pose_ = *msg;

	geometry_msgs::PoseStamped msg_poseStamped;
	msg_poseStamped.pose = msg_pose_;
	msg_poseStamped.header.stamp = ros::Time::now();
	pub_curr_pos_.publish(msg_poseStamped);
}

void abb_bridge::callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg)
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

bool abb_bridge::callback_robot_curr_pose(rpwc_msgs::robotArmState::Request  &req, rpwc_msgs::robotArmState::Response &res)
{
	res.pose.pose = msg_pose_;
	res.pose.header.stamp = ros::Time::now();
	return true;
}

bool abb_bridge::callback_switch_controller(rpwc_bridge::set_controller::Request  &req, rpwc_bridge::set_controller::Response &res)
{
	abb_driver::srv_abb_controller switch_controller;
	bool flag_switch;

	std::string tmp_name = req.controller_name.data;
	if(tmp_name.compare("pos_ctr") == 0) 
	{
		switch_controller.request.controller = 0;
		flag_switch = true;
	}
	else if(tmp_name.compare("grav_ctr") == 0)
	{
		switch_controller.request.controller = 2;
		flag_switch = true;
	}
	else flag_switch = false;


	if(flag_switch)
	{
		if (!client_switch_controller_.call(switch_controller)) ROS_ERROR("Failed to call service client_switch_controller_ ");
		res.answer.data = true;
	}
	else
	{
		std::cout << "CONTROLLER NOT AVAILABLE" << std::endl; 
		res.answer.data = false;
	}

	return true;
}

