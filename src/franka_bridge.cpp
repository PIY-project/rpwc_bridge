#include <franka_bridge.h>

franka_bridge::franka_bridge()
{
	//Subscriber
	sub_curr_pos_ = n_.subscribe("cartesian_impedance_controller_softbots_stiff_matrix/franka_ee_pose", 1, &franka_bridge::callback_curr_pose, this);
	sub_curr_pos_gravity_ = n_.subscribe("gravity_comp/franka_ee_pose", 1, &franka_bridge::callback_curr_pose_gravity, this);
	sub_equi_pos_ = n_.subscribe("cartesian_impedance_controller_softbots_stiff_matrix/equilibrium_pose", 1, &franka_bridge::callback_equi_pose, this);

	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &franka_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::PoseStamped>("cartesian_impedance_controller_softbots_stiff_matrix/equilibrium_pose", 1);
    pub_curr_pos_ = n_.advertise<geometry_msgs::PoseStamped>("rpwc_robot_curr_pose", 1);
    pub_rec_pos_ = n_.advertise<geometry_msgs::PoseStamped>("rpwc_robot_rec_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &franka_bridge::callback_robot_curr_pose, this);
  	
  	//server_switch_controller_ = n_.advertiseService("rpwc_switch_arm_controller", &franka_bridge::callback_switch_controller, this);
	srv_controller_ = n_.advertiseService("rpwc_controller", &franka_bridge::callback_controller, this);
  	client_switch_controller_ = n_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

	old_controller_name_ = "cartesian_impedance_controller_softbots_stiff_matrix";

	first_quat_base_EE_ = first_rec_ = first_quat_base_EE_gravity_ = true;
}

franka_bridge::~franka_bridge()
{

}

void franka_bridge::callback_curr_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	quat_base2EE_.vec() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
	quat_base2EE_.w() = msg->pose.orientation.w;
	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
	if(first_quat_base_EE_)
	{
		first_quat_base_EE_= false;
		quat_base2EE_old_ = quat_base2EE_;
	} 

	double sign_check = quat_base2EE_.w() * quat_base2EE_old_.w() + quat_base2EE_.x() * quat_base2EE_old_.x() + quat_base2EE_.y() * quat_base2EE_old_.y() + quat_base2EE_.z() * quat_base2EE_old_.z();
	if(sign_check < 0.0)
	{
		quat_base2EE_.w() = quat_base2EE_.w() * (-1); 
		quat_base2EE_.vec() = quat_base2EE_.vec() * (-1); 
	}
	quat_base2EE_old_ = quat_base2EE_;

	msg_pose_.pose.orientation.w = quat_base2EE_.w();
	msg_pose_.pose.orientation.x = quat_base2EE_.x();
	msg_pose_.pose.orientation.y = quat_base2EE_.y();
	msg_pose_.pose.orientation.z = quat_base2EE_.z();
	msg_pose_.pose.position.x = msg->pose.position.x;
	msg_pose_.pose.position.y = msg->pose.position.y;
	msg_pose_.pose.position.z = msg->pose.position.z;
	msg_pose_.header.stamp = ros::Time::now();
	pub_curr_pos_.publish(msg_pose_);
}

void franka_bridge::callback_curr_pose_gravity(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	quat_base2EE_gravity_.vec() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
	quat_base2EE_gravity_.w() = msg->pose.orientation.w;
	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
	if(first_quat_base_EE_gravity_)
	{
		first_quat_base_EE_gravity_= false;
		quat_base2EE_gravity_old_ = quat_base2EE_gravity_;
	} 

	double sign_check = quat_base2EE_gravity_.w() * quat_base2EE_gravity_old_.w() + quat_base2EE_gravity_.x() * quat_base2EE_gravity_old_.x() + quat_base2EE_gravity_.y() * quat_base2EE_gravity_old_.y() + quat_base2EE_gravity_.z() * quat_base2EE_gravity_old_.z();
	if(sign_check < 0.0)
	{
		quat_base2EE_gravity_.w() = quat_base2EE_gravity_.w() * (-1); 
		quat_base2EE_gravity_.vec() = quat_base2EE_gravity_.vec() * (-1); 
	}
	quat_base2EE_gravity_old_ = quat_base2EE_gravity_;

	msg_pose_gravity_.pose.orientation.w = quat_base2EE_gravity_.w();
	msg_pose_gravity_.pose.orientation.x = quat_base2EE_gravity_.x();
	msg_pose_gravity_.pose.orientation.y = quat_base2EE_gravity_.y();
	msg_pose_gravity_.pose.orientation.z = quat_base2EE_gravity_.z();
	msg_pose_gravity_.pose.position.x = msg->pose.position.x;
	msg_pose_gravity_.pose.position.y = msg->pose.position.y;
	msg_pose_gravity_.pose.position.z = msg->pose.position.z;
	msg_pose_gravity_.header.stamp = ros::Time::now();
}

void franka_bridge::callback_equi_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	quat_rec_.vec() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z;
	quat_rec_.w() = msg->pose.orientation.w;
	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
	if(first_rec_)
	{
		first_rec_= false;
		quat_rec_old_ = quat_rec_;
	} 

	double sign_check = quat_rec_.w() * quat_rec_old_.w() + quat_rec_.x() * quat_rec_old_.x() + quat_rec_.y() * quat_rec_old_.y() + quat_rec_.z() * quat_rec_old_.z();
	if(sign_check < 0.0)
	{
		quat_rec_.w() = quat_rec_.w() * (-1); 
		quat_rec_.vec() = quat_rec_.vec() * (-1); 
	}
	quat_rec_old_ = quat_rec_;

	msg_pose_.pose.orientation.w = quat_rec_.w();
	msg_pose_.pose.orientation.x = quat_rec_.x();
	msg_pose_.pose.orientation.y = quat_rec_.y();
	msg_pose_.pose.orientation.z = quat_rec_.z();
	msg_pose_.pose.position.x = msg->pose.position.x;
	msg_pose_.pose.position.y = msg->pose.position.y;
	msg_pose_.pose.position.z = msg->pose.position.z;
	msg_pose_.header.stamp = ros::Time::now();
	pub_rec_pos_.publish(msg_pose_);
}

void franka_bridge::callback_rpwc_pose_des(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped send_pose;
	send_pose.pose.position.x = msg->pose.position.x;
	send_pose.pose.position.y = msg->pose.position.y;
	send_pose.pose.position.z = msg->pose.position.z;
	send_pose.pose.orientation.w = msg->pose.orientation.w;
  	send_pose.pose.orientation.x = msg->pose.orientation.x;
  	send_pose.pose.orientation.y = msg->pose.orientation.y;
  	send_pose.pose.orientation.z = msg->pose.orientation.z;
	pub_pos_des_.publish(send_pose);
}

bool franka_bridge::callback_robot_curr_pose(rpwc_msgs::robotArmState::Request  &req, rpwc_msgs::robotArmState::Response &res)
{
	msg_pose_.header.stamp = ros::Time::now();
	if(old_controller_name_.compare("cartesian_impedance_controller_softbots_stiff_matrix") == 0) res.pose = msg_pose_;
	else res.pose = msg_pose_gravity_;
	
	return true;
}

bool franka_bridge::callback_controller(rpwc_msgs::setController::Request  &req, rpwc_msgs::setController::Response &res)
{
	std::string controller_name;
	if(req.controller == 0) controller_name = "cartesian_impedance_controller_softbots_stiff_matrix";
	else if(req.controller == 1) controller_name = "joint_position_one_task_inv_kin";
	else if(req.controller == 2) controller_name = "gravity_comp";

	if(old_controller_name_.compare(controller_name) == 0)
	{
		ROS_WARN_STREAM("THE CONTROLLER IS ALREADY SETTED TO: " << controller_name);
	}
	else
	{
		controller_manager_msgs::SwitchController switch_controller;
		switch_controller.request.start_controllers.clear();
		switch_controller.request.stop_controllers.clear();

		switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

		switch_controller.request.start_controllers.push_back("");
		switch_controller.request.stop_controllers.push_back(old_controller_name_);
		if (!client_switch_controller_.call(switch_controller)) ROS_ERROR("Failed to call service client_switch_controller_ ");
		switch_controller.request.start_controllers.push_back(controller_name);
		switch_controller.request.stop_controllers.push_back("");
		if (!client_switch_controller_.call(switch_controller)) ROS_ERROR("Failed to call service client_switch_controller_ ");

		old_controller_name_ = controller_name;
	}

	

	return true;
}