#include <franka_bridge.h>

franka_bridge::franka_bridge()
{
	std::string topic_franka_states, topic_pose_des;
	n_.getParam("topic_franka_states", topic_franka_states);
	n_.getParam("topic_pose_des", topic_pose_des);
	//Subscriber
	sub_curr_pos_ = n_.subscribe(topic_franka_states, 1, &franka_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &franka_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::PoseStamped>(topic_pose_des, 1);
    pub_curr_pos_ = n_.advertise<geometry_msgs::Pose>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &franka_bridge::callback_robot_curr_pose, this);
  	
  	server_switch_controller_ = n_.advertiseService("rpwc_switch_arm_controller", &franka_bridge::callback_switch_controller, this);
  	client_switch_controller_ = n_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

	T_base_2_EE_ = Eigen::Matrix4d::Identity();
	old_controller_name_ = "joint_position_one_task_inv_kin";

	first_quat_base_EE_ = true;
}

franka_bridge::~franka_bridge()
{

}

void franka_bridge::callback_curr_pose(const franka_msgs::FrankaState::ConstPtr& msg)
{
	T_base_2_EE_(0,0) = msg->O_T_EE[0];
	T_base_2_EE_(1,0) = msg->O_T_EE[1];
	T_base_2_EE_(2,0) = msg->O_T_EE[2];
	T_base_2_EE_(3,0) = msg->O_T_EE[3];
	T_base_2_EE_(0,1) = msg->O_T_EE[4];
	T_base_2_EE_(1,1) = msg->O_T_EE[5];
	T_base_2_EE_(2,1) = msg->O_T_EE[6];
	T_base_2_EE_(3,1) = msg->O_T_EE[7];
	T_base_2_EE_(0,2) = msg->O_T_EE[8];
	T_base_2_EE_(1,2) = msg->O_T_EE[9];
	T_base_2_EE_(2,2) = msg->O_T_EE[10];
	T_base_2_EE_(3,2) = msg->O_T_EE[11];
	T_base_2_EE_(0,3) = msg->O_T_EE[12];
	T_base_2_EE_(1,3) = msg->O_T_EE[13];
	T_base_2_EE_(2,3) = msg->O_T_EE[14];
	T_base_2_EE_(3,3) = msg->O_T_EE[15];
	quat_base2EE_ = T_base_2_EE_.block<3,3>(0,0);
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

	geometry_msgs::Pose msg_pose;
	msg_pose.orientation.w = quat_base2EE_.w();
	msg_pose.orientation.x = quat_base2EE_.x();
	msg_pose.orientation.y = quat_base2EE_.y();
	msg_pose.orientation.z = quat_base2EE_.z();
	msg_pose.position.x = T_base_2_EE_(0,3);
	msg_pose.position.y = T_base_2_EE_(1,3);
	msg_pose.position.z = T_base_2_EE_(2,3);
	pub_curr_pos_.publish(msg_pose);
}

void franka_bridge::callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg)
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

bool franka_bridge::callback_robot_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res)
{
	geometry_msgs::Pose robot_curr_pose;
	robot_curr_pose.orientation.w = quat_base2EE_.w();
	robot_curr_pose.orientation.x = quat_base2EE_.x();
	robot_curr_pose.orientation.y = quat_base2EE_.y();
	robot_curr_pose.orientation.z = quat_base2EE_.z();
	robot_curr_pose.position.x = T_base_2_EE_(0,3);
	robot_curr_pose.position.y = T_base_2_EE_(1,3);
	robot_curr_pose.position.z = T_base_2_EE_(2,3);

	res.robot_curr_pose = robot_curr_pose;
	return true;
}

bool franka_bridge::callback_switch_controller(rpwc_bridge::set_controller::Request  &req, rpwc_bridge::set_controller::Response &res)
{
	std::string tmp_name = req.controller_name.data;
	if(tmp_name.compare("pos_ctr") == 0) controller_name_ = "joint_position_one_task_inv_kin";
	else if(tmp_name.compare("imp_ctr") == 0) controller_name_ = "cartesian_impedance_controller_softbots_stiff_matrix";
	else if(tmp_name.compare("grav_ctr") == 0) controller_name_ = "gravity_comp";
	// controller_name_ = req.controller_name.data;





	controller_manager_msgs::SwitchController switch_controller;
    switch_controller.request.start_controllers.clear();
    switch_controller.request.stop_controllers.clear();

    switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

    switch_controller.request.start_controllers.push_back("");
    switch_controller.request.stop_controllers.push_back(old_controller_name_);
    if (!client_switch_controller_.call(switch_controller)) ROS_ERROR("Failed to call service client_switch_controller_ ");
    switch_controller.request.start_controllers.push_back(controller_name_);
    switch_controller.request.stop_controllers.push_back("");
    if (!client_switch_controller_.call(switch_controller)) ROS_ERROR("Failed to call service client_switch_controller_ ");

    old_controller_name_ = controller_name_;

	res.answer.data = true;
	return true;
}

