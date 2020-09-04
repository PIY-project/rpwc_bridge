
#include <ros/ros.h>
#include <ros/rate.h>
#include <rpwc/rpwc_gripper_cmd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>

bool new_gripper_cmd_ = false;
double gripper_cmd_;

bool callback_rpwc_gripper_cmd(rpwc::rpwc_gripper_cmd::Request  &req, rpwc::rpwc_gripper_cmd::Response &res)
{
	gripper_cmd_ = req.EE_cmd_.data;
	new_gripper_cmd_ = true;
	return true;
}


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

	ros::init(argc, argv, "franka_gripper_bridge_node");
	ros::NodeHandle nh;

  	double rate_50Hz = 50.0;
	ros::Rate r_50HZ(rate_50Hz);

	//Service Server
  	ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("/rpwc_gripper_cmd", &callback_rpwc_gripper_cmd);
  	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp("grasp", true);
	actionlib::SimpleActionClient<franka_gripper::HomingAction> ac_homing("homing", true);
	ac_grasp.waitForServer(); //will wait for infinite time
	ac_homing.waitForServer(); //will wait for infinite time
	franka_gripper::GraspGoal grasp_goal;
	grasp_goal.width = 0.01;
	grasp_goal.epsilon.inner = 0.005;
	grasp_goal.epsilon.outer = 0.085;
	grasp_goal.speed = 0.1;
	grasp_goal.force = 1;

	franka_gripper::HomingGoal homing_goal;

	while(ros::ok())
	{
		if(new_gripper_cmd_)
		{
			new_gripper_cmd_ = false;
			// call action to the gripper package
			if(gripper_cmd_>= 0.5)
			{
				ac_grasp.sendGoal(grasp_goal);
				// bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
			}
			else
			{
				ac_homing.sendGoal(homing_goal);
			}
		}
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

