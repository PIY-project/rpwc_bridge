
#include <ros/ros.h>
#include <ros/rate.h>
#include <std_msgs/Float64.h>
#include <onrobot_msgs/onrobot_gripper.h>
#include <rpwc_msgs/RobotEeStateStamped.h>



ros::Publisher pub_gripper_des_, pub_for_recording_;
bool grasping_ = false;
onrobot_msgs::onrobot_gripper cmd_msg_;



void callback_rpwc_gripper_cmd(const std_msgs::Float64::ConstPtr& msg)
{
	cmd_msg_.position = int(45-(msg->data*45));

	rpwc_msgs::RobotEeStateStamped tmpMsg;
	tmpMsg.data.data = msg->data;
	tmpMsg.header.stamp = ros::Time::now();

	pub_gripper_des_.publish(cmd_msg_);
	pub_for_recording_.publish(tmpMsg);
}


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

	ros::init(argc, argv, "onrobot_twofg_bridge_node");
	ros::NodeHandle nh;


  	double rate_30Hz = 30.0;
	ros::Rate r_30HZ(rate_30Hz);

	cmd_msg_.force = 20;
	cmd_msg_.velocity = 80;

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
  	//Publisher
    pub_gripper_des_ = nh.advertise<onrobot_msgs::onrobot_gripper>("set_width", 1);
	pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);

	while(ros::ok())
	{
		ros::spinOnce();
		r_30HZ.sleep();
	}// end while()
	return 0;
}
