#include <ur_bridge.h>

ur_bridge::ur_bridge()
{
	std::string topic_ur_states, topic_pose_des;
	n_.getParam("topic_ur_states", topic_ur_states);
	n_.getParam("topic_pose_des", topic_pose_des);
	//Subscriber
	sub_curr_pos_ = n_.subscribe(topic_ur_states, 1, &ur_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &ur_bridge::callback_rpwc_pose_des, this);



	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::Pose>(topic_pose_des, 1);
    pub_pos_des_pass_obs_ = n_.advertise<geometry_msgs::Pose>("rpwc_pose_des_pass_obs", 1);
    pub_curr_pos_ = n_.advertise<geometry_msgs::Pose>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &ur_bridge::callback_robot_curr_pose, this);

 	// PASS OBS
	sub_rpwc_pass_obs_ = n_.subscribe("test_pass_obs", 1, &ur_bridge::callback_rpwc_pass_obs, this);
  	server_pass_obs_curr_pose_ = n_.advertiseService("rpwc_pass_obs_curr_pose", &ur_bridge::callback_pass_obs_curr_pose, this);

	first_quat_base_EE_ = first_quat_base_EE_pass_obs_ = true;
	T_base_2_EE_ = Eigen::Matrix4d::Identity();
}

ur_bridge::~ur_bridge()
{

}


void ur_bridge::callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
	quat_base2EE_.w() = msg->orientation.w;
	quat_base2EE_.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;

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

	T_base_2_EE_(0,3) = msg->position.x;
	T_base_2_EE_(1,3) = msg->position.y;
	T_base_2_EE_(2,3) = msg->position.z;
	Eigen::Matrix3d R_tmp(quat_base2EE_);
	T_base_2_EE_.block<3,3>(0,0) = R_tmp;

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

void ur_bridge::callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg)
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

bool ur_bridge::callback_robot_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res)
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

void ur_bridge::callback_rpwc_pass_obs(const geometry_msgs::Pose::ConstPtr& msg)
{
	// geometry_msgs::Pose send_pose;
	// send_pose.position.x = msg->position.x;
	// send_pose.position.y = msg->position.y;
	// send_pose.position.z = msg->position.z;
	// send_pose.orientation.w = msg->orientation.w;
 //  	send_pose.orientation.x = msg->orientation.x;
 //  	send_pose.orientation.y = msg->orientation.y;
 //  	send_pose.orientation.z = msg->orientation.z;
	// pub_pos_des_.publish(send_pose);
	quat_base2EE_pass_obs_.w() = msg->orientation.w;
	quat_base2EE_pass_obs_.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;

	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
	if(first_quat_base_EE_pass_obs_)
	{
		first_quat_base_EE_pass_obs_= false;
		quat_base2EE_pass_obs_old_ = quat_base2EE_pass_obs_;
	} 

	double sign_check = quat_base2EE_pass_obs_.w() * quat_base2EE_pass_obs_old_.w() + quat_base2EE_pass_obs_.x() * quat_base2EE_pass_obs_old_.x() + quat_base2EE_pass_obs_.y() * quat_base2EE_pass_obs_old_.y() + quat_base2EE_pass_obs_.z() * quat_base2EE_pass_obs_old_.z();
	if(sign_check < 0.0)
	{
		quat_base2EE_pass_obs_.w() = quat_base2EE_pass_obs_.w() * (-1); 
		quat_base2EE_pass_obs_.vec() = quat_base2EE_pass_obs_.vec() * (-1); 
	}
	quat_base2EE_pass_obs_old_ = quat_base2EE_pass_obs_;

	T_base_2_EE_pass_obs_(0,3) = msg->position.x;
	T_base_2_EE_pass_obs_(1,3) = msg->position.y;
	T_base_2_EE_pass_obs_(2,3) = msg->position.z;
	Eigen::Matrix3d R_tmp(quat_base2EE_pass_obs_);
	T_base_2_EE_pass_obs_.block<3,3>(0,0) = R_tmp;

	geometry_msgs::Pose msg_pose;
	msg_pose.orientation.w = quat_base2EE_pass_obs_.w();
	msg_pose.orientation.x = quat_base2EE_pass_obs_.x();
	msg_pose.orientation.y = quat_base2EE_pass_obs_.y();
	msg_pose.orientation.z = quat_base2EE_pass_obs_.z();
	msg_pose.position.x = T_base_2_EE_pass_obs_(0,3);
	msg_pose.position.y = T_base_2_EE_pass_obs_(1,3);
	msg_pose.position.z = T_base_2_EE_pass_obs_(2,3);
	pub_pos_des_pass_obs_.publish(msg_pose);

}

bool ur_bridge::callback_pass_obs_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res)
{
	Eigen::Quaterniond q_tmp(T_base_2_EE_pass_obs_.block<3,3>(0,0));
	geometry_msgs::Pose robot_curr_pose;
	robot_curr_pose.orientation.w = q_tmp.w();
	robot_curr_pose.orientation.x = q_tmp.x();
	robot_curr_pose.orientation.y = q_tmp.y();
	robot_curr_pose.orientation.z = q_tmp.z();
	robot_curr_pose.position.x = T_base_2_EE_pass_obs_(0,3);
	robot_curr_pose.position.y = T_base_2_EE_pass_obs_(1,3);
	robot_curr_pose.position.z = T_base_2_EE_pass_obs_(2,3);

	res.robot_curr_pose = robot_curr_pose;
	return true;
}