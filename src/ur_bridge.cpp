#include <ur_bridge.h>

ur_bridge::ur_bridge()
{
	std::string topic_ur_states, topic_pose_des;
	n_.getParam("topic_ur_states", topic_ur_states);
	n_.getParam("topic_pose_des", topic_pose_des);
	//Subscriber
	sub_curr_pos_ = n_.subscribe(topic_ur_states, 1, &ur_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("/rpwc_pose_des", 1, &ur_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::Pose>(topic_pose_des, 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("/rpwc_robot_curr_pose", &ur_bridge::callback_robot_curr_pose, this);

	first_quat_base_EE_ = true;
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