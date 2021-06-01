#include <ros/ros.h>
#include <franka_msgs/FrankaState.h>
#include <rpwc/robot_curr_pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <rpwc/rpwc_gripper_cmd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

#include <eigen3/Eigen/Eigen>

typedef actionlib::SimpleActionClient<franka_gripper::GraspAction> ClientGrasp;
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction> ClientMove;

class franka_gripper_bridge
{
public:
	franka_gripper_bridge() : ac_grasp("franka_gripper/grasp", true), ac_move("franka_gripper/move", true)
	{
		grasping_ = false;
		ac_grasp.waitForServer();
		ac_move.waitForServer();
		grasp_goal_.width = 0.01;

		grasp_goal_.epsilon.inner = 0.005;
		grasp_goal_.epsilon.outer = 0.085;
		grasp_goal_.speed = 0.5;
		grasp_goal_.force = 60;

		move_goal_.width = 0.09;
		move_goal_.speed = 0.1;
		//Subscriber
		sub_rpwc_gripper_cmd_ = n_.subscribe("rpwc_EE_cmd", 1, &franka_gripper_bridge::callback_rpwc_gripper_cmd, this);
		//Service Server
  		server_rpwc_gripper_cmd_ = n_.advertiseService("rpwc_EE_single_cmd", &franka_gripper_bridge::callback_rpwc_gripper_single_cmd, this);
  		server_rpwc_gripper_single_move_cmd_ = n_.advertiseService("rpwc_EE_single_move_cmd", &franka_gripper_bridge::callback_rpwc_gripper_single_move_cmd, this);
	}
	~franka_gripper_bridge();


  double dt_;

private:
	void callback_rpwc_gripper_cmd(const std_msgs::Float64::ConstPtr& msg);
	bool callback_rpwc_gripper_single_cmd(rpwc::rpwc_gripper_cmd::Request  &req, rpwc::rpwc_gripper_cmd::Response &res);
	bool callback_rpwc_gripper_single_move_cmd(rpwc::rpwc_gripper_cmd::Request  &req, rpwc::rpwc_gripper_cmd::Response &res);


	ros::NodeHandle n_;

	ros::Subscriber sub_rpwc_gripper_cmd_;
	ros::ServiceServer server_rpwc_gripper_cmd_, server_rpwc_gripper_single_move_cmd_;
	ClientGrasp ac_grasp;
	ClientMove ac_move;
	franka_gripper::GraspGoal grasp_goal_;
	franka_gripper::MoveGoal move_goal_;

	bool grasping_;

};//End of class SubscribeAndPublish