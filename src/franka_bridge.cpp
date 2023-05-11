#include <franka_bridge.h>

franka_bridge::franka_bridge()
{
	//Subscriber
	sub_curr_pos_ = n_.subscribe("franka_ee_pose", 1, &franka_bridge::callback_curr_pose, this);
	

	// sub_curr_pos_ = n_.subscribe("franka_state_controller/franka_states", 1, &franka_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &franka_bridge::callback_rpwc_pose_des, this);
	
	
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::Pose>("pose_des", 1);
    pub_curr_pos_ = n_.advertise<geometry_msgs::PoseStamped>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &franka_bridge::callback_robot_curr_pose, this);
  	
  	//server_switch_controller_ = n_.advertiseService("rpwc_switch_arm_controller", &franka_bridge::callback_switch_controller, this);
  	client_switch_controller_ = n_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");

	old_controller_name_ = "joint_position_one_task_inv_kin";

	first_quat_base_EE_ = true;
}

franka_bridge::~franka_bridge()
{

}

void franka_bridge::callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
	quat_base2EE_.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;
	quat_base2EE_.w() = msg->orientation.w;
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
	msg_pose_.pose.position.x = msg->position.x;
	msg_pose_.pose.position.y = msg->position.y;
	msg_pose_.pose.position.z = msg->position.z;
	msg_pose_.header.stamp = ros::Time::now();
	pub_curr_pos_.publish(msg_pose_);
}

void franka_bridge::callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg)
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

bool franka_bridge::callback_robot_curr_pose(rpwc_msgs::robotArmState::Request  &req, rpwc_msgs::robotArmState::Response &res)
{
	msg_pose_.header.stamp = ros::Time::now();
	res.pose = msg_pose_;
	return true;
}
