#include <abb_bridge.h>

abb_bridge::abb_bridge()
{
	//Subscriber
	sub_curr_pos_ = n_.subscribe("curr_pose", 1, &abb_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &abb_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::PoseStamped>("pose_des", 1);
    pub_curr_pos_ = n_.advertise<geometry_msgs::Pose>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &abb_bridge::callback_robot_curr_pose, this);
  	
  	// server_switch_controller_ = n_.advertiseService("rpwc_switch_arm_controller", &abb_bridge::callback_switch_controller, this);
  	// client_switch_controller_ = n_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

	T_base_2_EE_ = Eigen::Matrix4d::Identity();
	old_controller_name_ = "joint_position_one_task_inv_kin";

	first_quat_base_EE_ = true;
}

abb_bridge::~abb_bridge()
{

}

void abb_bridge::callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
	msg_pose_ = *msg;
	pub_curr_pos_.publish(msg_pose_);
}

void abb_bridge::callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg)
{
	geometry_msgs::PoseStamped send_pose;
	send_pose.pose.position.x = msg->position.x;
	send_pose.pose.position.y = msg->position.y;
	send_pose.pose.position.z = msg->position.z;
	send_pose.pose.orientation.w = msg->orientation.w;
  	send_pose.pose.orientation.x = msg->orientation.x;
  	send_pose.pose.orientation.y = msg->orientation.y;
  	send_pose.pose.orientation.z = msg->orientation.z;
  	send_pose.header.stamp = ros::Time::now();
	pub_pos_des_.publish(send_pose);
}

bool abb_bridge::callback_robot_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res)
{
	res.robot_curr_pose = msg_pose_;
	return true;
}

// bool abb_bridge::callback_switch_controller(rpwc_bridge::set_controller::Request  &req, rpwc_bridge::set_controller::Response &res)
// {
// 	std::string tmp_name = req.controller_name.data;
// 	if(tmp_name.compare("pos_ctr") == 0) controller_name_ = "joint_position_one_task_inv_kin";
// 	else if(tmp_name.compare("imp_ctr") == 0) controller_name_ = "cartesian_impedance_modified";
// 	else if(tmp_name.compare("grav_ctr") == 0) controller_name_ = "gravity_comp";
// 	// controller_name_ = req.controller_name.data;





// 	controller_manager_msgs::SwitchController switch_controller;
//     switch_controller.request.start_controllers.clear();
//     switch_controller.request.stop_controllers.clear();

//     switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

//     switch_controller.request.start_controllers.push_back("");
//     switch_controller.request.stop_controllers.push_back(old_controller_name_);
//     if (!client_switch_controller_.call(switch_controller)) ROS_ERROR("Failed to call service client_switch_controller_ ");
//     switch_controller.request.start_controllers.push_back(controller_name_);
//     switch_controller.request.stop_controllers.push_back("");
//     if (!client_switch_controller_.call(switch_controller)) ROS_ERROR("Failed to call service client_switch_controller_ ");

//     old_controller_name_ = controller_name_;

// 	res.answer.data = true;
// 	return true;
// }

