#include <test_abb_bridge.h>

test_abb_bridge::test_abb_bridge()
{
	// //Subscriber
	sub_curr_pos_ = n_.subscribe("curr_pose", 1, &test_abb_bridge::callback_curr_pose, this);
	//Service Server
  	server_switch_controller_ = n_.advertiseService("rpwc_switch_arm_controller", &test_abb_bridge::callback_switch_controller, this);
  	//Service Client
  	client_switch_controller_ = n_.serviceClient<abb_driver::srv_abb_controller>("abb_controller");

  	vec_ctr_name_.clear();
  	vec_ctr_name_.push_back("pos_ctr");
  	vec_ctr_name_.push_back("grav_ctr");
}

test_abb_bridge::~test_abb_bridge()
{

}

void test_abb_bridge::callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg)
{
	quat_base_2_lastlink_.w() = msg->orientation.w;
	quat_base_2_lastlink_.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;
	
	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
	if(first_quat_base_2_lastlink_)
	{
		first_quat_base_2_lastlink_= false;
		quat_base_2_lastlink_old_ = quat_base_2_lastlink_;
	} 

	double sign_check = quat_base_2_lastlink_.w() * quat_base_2_lastlink_old_.w() + quat_base_2_lastlink_.x() * quat_base_2_lastlink_old_.x() + quat_base_2_lastlink_.y() * quat_base_2_lastlink_old_.y() + quat_base_2_lastlink_.z() * quat_base_2_lastlink_old_.z();
	if(sign_check < 0.0)
	{
		quat_base_2_lastlink_.w() = quat_base_2_lastlink_.w() * (-1); 
		quat_base_2_lastlink_.vec() = quat_base_2_lastlink_.vec() * (-1); 
	}
	quat_base_2_lastlink_old_ = quat_base_2_lastlink_;

	T_base_2_lastlink_.block<3,3>(0,0) = Eigen::Matrix3d(quat_base_2_lastlink_);
	T_base_2_lastlink_(0,3) = msg->position.x;
	T_base_2_lastlink_(1,3) = msg->position.y;
	T_base_2_lastlink_(2,3) = msg->position.z;


	geometry_msgs::Pose msg_pose;
	msg_pose.orientation.w = quat_base_2_lastlink_.w();
	msg_pose.orientation.x = quat_base_2_lastlink_.x();
	msg_pose.orientation.y = quat_base_2_lastlink_.y();
	msg_pose.orientation.z = quat_base_2_lastlink_.z();
	msg_pose.position.x = T_base_2_lastlink_(0,3);
	msg_pose.position.y = T_base_2_lastlink_(1,3);
	msg_pose.position.z = T_base_2_lastlink_(2,3);
	pub_rpwc_curr_pose_.publish(msg_pose);
}


bool test_abb_bridge::callback_switch_controller(rpwc_bridge::set_controller::Request  &req, rpwc_bridge::set_controller::Response &res)
{
	abb_driver::srv_abb_controller switch_controller;
	bool flag_switch  = false;

	for(int i = 0; i < vec_ctr_name_.size(); i++)
	{
		std::string tmp_name = req.controller_name.data;
		if(tmp_name.compare(vec_ctr_name_[i]) == 0)
		{
			flag_switch = true;
			break;
		}
	}

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

