#include <franka_gripper_bridge.h>


franka_gripper_bridge::~franka_gripper_bridge()
{

}

void franka_gripper_bridge::callback_rpwc_gripper_cmd(const std_msgs::Float64::ConstPtr& msg)
{
	if(msg->data >= 0.5 && !grasping_)
	{
		grasping_ = true;
		ac_grasp.sendGoal(grasp_goal_);
	}
	else if(msg->data < 0.5 && grasping_)
	{
		grasping_ = false;
		ac_move.sendGoal(move_goal_);
	}
}

bool franka_gripper_bridge::callback_rpwc_gripper_single_cmd(rpwc::rpwc_gripper_cmd::Request  &req, rpwc::rpwc_gripper_cmd::Response &res)
{
	if(req.EE_cmd_.data >= 0.5 && !grasping_)
	{
		grasping_ = true;
		ac_grasp.sendGoal(grasp_goal_);
	}
	else if(req.EE_cmd_.data < 0.5 && grasping_)
	{
		grasping_ = false;
		ac_move.sendGoal(move_goal_);
	}
	return true;
}