
#include <ros/ros.h>
#include <ros/rate.h>
#include "franka_gripper_bridge.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

	ros::init(argc, argv, "franka_gripper_bridge_node");
	ros::NodeHandle nh;
	franka_gripper_bridge Obj;
  	double rate_50Hz = 50.0;
	ros::Rate r_50HZ(rate_50Hz);

	// franka_gripper::HomingGoal homing_goal;

	while(ros::ok())
	{
		// if(new_gripper_cmd_)
		// {
		// 	new_gripper_cmd_ = false;
		// 	// call action to the gripper package
		// 	if(gripper_cmd_>= 0.5)
		// 	{
		// 		ac_grasp.sendGoal(grasp_goal_);
		// 		// bool finished_before_timeout = ac_grasp.waitForResult(ros::Duration(5.0));
		// 		// if(finished_before_timeout)std::cout<< "finito"<<std::endl;
		// 		// else std::cout<< "noooooooooooooon finito"<<std::endl;
		// 	}
		// 	else
		// 	{
		// 		ac_move.sendGoal(move_goal_);
		// 		// ac_homing.sendGoal(homing_goal);
		// 		// std::cout<< "homing sent"<<std::endl;

		// 	}
		// }
		ros::spinOnce();
		r_50HZ.sleep();
	}// end while()
	return 0;
}

