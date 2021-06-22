
#include <ros/ros.h>
#include <ros/rate.h>
#include <rpwc/rpwc_gripper_cmd.h>
#include <std_msgs/Float64.h>


ros::Publisher pub_hand_des_;

bool callback_rpwc_gripper_single_cmd(rpwc::rpwc_gripper_cmd::Request  &req, rpwc::rpwc_gripper_cmd::Response &res)
{
  	std_msgs::Float64 hand_cmd;
  	hand_cmd.data = req.EE_cmd_.data;
  	pub_hand_des_.publish(hand_cmd);

	return true;
}

void callback_rpwc_gripper_cmd(const std_msgs::Float64::ConstPtr& msg)
{
	//HAND CLOSURE
  	std_msgs::Float64 hand_cmd;
  	hand_cmd.data = msg->data;
  	pub_hand_des_.publish(hand_cmd);
}


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

	ros::init(argc, argv, "softhand_bridge_node");
	ros::NodeHandle nh;

  	double rate_50Hz = 50.0;
	ros::Rate r_50HZ(rate_50Hz);

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
  	//Publisher
    pub_hand_des_ = nh.advertise<std_msgs::Float64>("SH_industry_cmd", 1);

	//Service Server
  	ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("rpwc_EE_single_cmd", &callback_rpwc_gripper_single_cmd);



	while(ros::ok())
	{
		
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

