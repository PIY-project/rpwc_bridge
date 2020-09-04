#include <franka_bridge.h>

franka_bridge::franka_bridge()
{
	std::string topic_franka_states, topic_pose_des;
	n_.getParam("topic_franka_states", topic_franka_states);
	n_.getParam("topic_pose_des", topic_pose_des);

	//Subscriber
	sub_curr_pos_ = n_.subscribe(topic_franka_states, 1, &franka_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("/rpwc_pose_des", 1, &franka_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::PoseStamped>(topic_pose_des, 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("/rpwc_robot_curr_pose", &franka_bridge::callback_robot_curr_pose, this);

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