
#include <ros/ros.h>
#include <ros/rate.h>
#include <rpwc/rpwc_gripper_cmd.h>
#include <std_msgs/Float64.h>
#include <onrobot_msgs/onrobot_gripper.h>



ros::Publisher pub_gripper_des_;
bool grasping_ = false;
onrobot_msgs::onrobot_gripper cmd_msg_;

bool callback_rpwc_gripper_single_cmd(rpwc::rpwc_gripper_cmd::Request  &req, rpwc::rpwc_gripper_cmd::Response &res)
{
	if(req.EE_cmd_.data >= 0.5 && !grasping_)
	{
		grasping_ = true;
		cmd_msg_.position = int(45-(req.EE_cmd_.data*45));
		pub_gripper_des_.publish(cmd_msg_);
	}
	else if(req.EE_cmd_.data < 0.5 && grasping_)
	{
		grasping_ = false;
		cmd_msg_.position = int(45-(req.EE_cmd_.data*45));
		pub_gripper_des_.publish(cmd_msg_);
	}

	return true;
}

void callback_rpwc_gripper_cmd(const std_msgs::Float64::ConstPtr& msg)
{
  	if(msg->data >= 0.5 && !grasping_)
	{
		grasping_ = true;
		cmd_msg_.position = int(0);
		pub_gripper_des_.publish(cmd_msg_);
	}
	else if(msg->data < 0.5 && grasping_)
	{
		grasping_ = false;
		cmd_msg_.position = int(45);
		pub_gripper_des_.publish(cmd_msg_);
	}
}


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

	ros::init(argc, argv, "onrobot_twofg_bridge_node");
	ros::NodeHandle nh;


  	double rate_50Hz = 50.0;
	ros::Rate r_50HZ(rate_50Hz);

	cmd_msg_.force = 60;
	cmd_msg_.velocity = 100;

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
  	//Publisher
    pub_gripper_des_ = nh.advertise<onrobot_msgs::onrobot_gripper>("set_width", 1);

	//Service Server
  	ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("rpwc_EE_single_cmd", &callback_rpwc_gripper_single_cmd);



	while(ros::ok())
	{
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}
