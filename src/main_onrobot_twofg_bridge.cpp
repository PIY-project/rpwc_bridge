#include <ros/ros.h>
#include <ros/rate.h>
#include <std_msgs/Float64.h>
#include <onrobot_msgs/onrobot_gripper.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/robotEeState.h>



ros::Publisher pub_gripper_des_, pub_for_recording_;
bool grasping_ = false;
onrobot_msgs::onrobot_gripper cmd_msg_;
rpwc_msgs::RobotEeStateStamped lastCmdMsg_;



void callback_rpwc_gripper_cmd(const std_msgs::Float64::ConstPtr& msg)
{
	cmd_msg_.position = int(45-(msg->data*45));

	lastCmdMsg_.data.data = msg->data;
	lastCmdMsg_.header.stamp = ros::Time::now();

	pub_gripper_des_.publish(cmd_msg_);
	pub_for_recording_.publish(lastCmdMsg_);
}

bool callback_robot_curr_pose(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	res.data = lastCmdMsg_.data;
	res.header = lastCmdMsg_.header;
	return true;
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
	lastCmdMsg_.data.data = 0;


	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
  	//Publisher
    pub_gripper_des_ = nh.advertise<onrobot_msgs::onrobot_gripper>("set_width", 1);
	pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
	//Service Server
  	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", &callback_robot_curr_pose);

	while(ros::ok())
	{
		ros::spinOnce();
		r_30HZ.sleep();
	}// end while()
	return 0;
}