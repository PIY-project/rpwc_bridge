#include <arm_base_bridge.h>

arm_base_bridge::arm_base_bridge()
{
	//Subscriber
	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &arm_base_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pose_des_ = n_.advertise<geometry_msgs::Pose>("pose_des", 1);
    pub_rpwc_curr_pose_ = n_.advertise<geometry_msgs::Pose>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &arm_base_bridge::callback_robot_curr_pose, this);

	T_base_2_lastlink_ = Eigen::Matrix4d::Identity();
	old_controller_name_ = "joint_position_one_task_inv_kin";
	first_quat_base_2_lastlink_ = true;
}

arm_base_bridge::~arm_base_bridge()
{

}

void arm_base_bridge::callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg)
{
	geometry_msgs::Pose send_pose;
	send_pose.position.x = msg->position.x;
	send_pose.position.y = msg->position.y;
	send_pose.position.z = msg->position.z;
	send_pose.orientation.w = msg->orientation.w;
  	send_pose.orientation.x = msg->orientation.x;
  	send_pose.orientation.y = msg->orientation.y;
  	send_pose.orientation.z = msg->orientation.z;
	pub_pose_des_.publish(send_pose);
}

bool arm_base_bridge::callback_robot_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res)
{
	geometry_msgs::Pose robot_curr_pose;
	robot_curr_pose.orientation.w = quat_base_2_lastlink_.w();
	robot_curr_pose.orientation.x = quat_base_2_lastlink_.x();
	robot_curr_pose.orientation.y = quat_base_2_lastlink_.y();
	robot_curr_pose.orientation.z = quat_base_2_lastlink_.z();
	robot_curr_pose.position.x = T_base_2_lastlink_(0,3);
	robot_curr_pose.position.y = T_base_2_lastlink_(1,3);
	robot_curr_pose.position.z = T_base_2_lastlink_(2,3);

	res.robot_curr_pose = robot_curr_pose;
	return true;
}

