#include <ros/ros.h>
#include <ros/rate.h>
#include <rpwc_msgs/robotEeCmdAction.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/robotEeState.h>
#include <rpwc_msgs/robotEeCmd.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <thread>
#include <iostream>


trajectory_msgs::JointTrajectory cmd_msg_;
ros::Publisher pub_for_recording_, pub_gripper_des_, pub_CommandHand_test_;
rpwc_msgs::RobotEeStateStamped lastCmdMsg_;
typedef actionlib::SimpleActionServer<rpwc_msgs::robotEeCmdAction> robotEeCmdAS;
double gripperFeedbackState_ = 0.0;

bool inizialized_ = false;

class eeCmdMove
{
	protected:
		robotEeCmdAS as_;
		rpwc_msgs::robotEeCmdGoalConstPtr ee_goal_;
		rpwc_msgs::robotEeCmdResult ee_result_;
		rpwc_msgs::robotEeCmdFeedback ee_feedback_;

public:
	eeCmdMove(ros::NodeHandle& nh_);
	
  	~eeCmdMove();

  void goal_callback();
  void preempt_callback();
};

eeCmdMove::eeCmdMove(ros::NodeHandle& nh_) : as_(nh_, "rpwc_EE_action_cmd", false)
{
	as_.registerGoalCallback(boost::bind(&eeCmdMove::goal_callback, this));
	as_.registerPreemptCallback(boost::bind(&eeCmdMove::preempt_callback, this));
	as_.start();
  	ROS_INFO_STREAM("eeCmdMove action server avviato.");
}

eeCmdMove::~eeCmdMove()
{
    ROS_INFO_STREAM("eeCmdMove distrutto.");
}

bool callback_server_rpwc_gripper_cmd(rpwc_msgs::robotEeCmd::Request  &req, rpwc_msgs::robotEeCmd::Response &res)
{
  	double position = req.state.position.data;
	if(req.state.position.data < 0.0)position = 0.0;
	else if (req.state.position.data > 1.0) position = 1.0;

	double velocity = 0;
	double force = 0;

	position = (position *1.5) - 0.5;
	cmd_msg_.joint_names.resize(1);
  	cmd_msg_.joint_names[0] = "qbsoftclaw_deflection_virtual_joint";
	cmd_msg_.points.clear();
  	cmd_msg_.points.resize(1);
  	cmd_msg_.points[0].positions.push_back(position);
  	cmd_msg_.points[0].time_from_start = ros::Duration(0.3);
	pub_gripper_des_.publish(cmd_msg_);

	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();

	return true;
}

void eeCmdMove::goal_callback()
{
    ee_goal_ = as_.acceptNewGoal();
	ROS_INFO_STREAM("Received goal with force: " << ee_goal_->state.force.data
                    << ", position: " << ee_goal_->state.position.data
                    << ", velocity: " << ee_goal_->state.velocity.data);

	rpwc_msgs::robotEeCmd::Request robotEeCmdSrv;
	robotEeCmdSrv.state = ee_goal_->state;
	rpwc_msgs::robotEeCmd::Response emptyResponse;

	if(fabs(lastCmdMsg_.position.data - ee_goal_->state.position.data) <= 0.30) 
	{
		ee_result_.success = true;
		ee_result_.msg = "Execution completed successfully.";
		as_.setSucceeded(ee_result_);
		return;
	}
	
	if(!callback_server_rpwc_gripper_cmd(robotEeCmdSrv, emptyResponse))
	{
		ee_result_.success = false;
		ee_result_.msg = "Error during command execution.";
		as_.setAborted(ee_result_);
		return;
	}

	ros::Duration(1.0).sleep(); //ensure the grip is closed

	if (ee_goal_->state.position.data > 0.0 && gripperFeedbackState_ < -0.59)
    {
		ee_result_.success = false;
		ee_result_.msg = "Error: gripper did not close properly.";
		as_.setAborted(ee_result_);
		return;
    }

	ee_result_.success = true;
	ee_result_.msg = "Execution completed successfully.";
	as_.setSucceeded(ee_result_);

	return;
}

void eeCmdMove::preempt_callback()
{
    ROS_WARN("Goal preempted");
    as_.setPreempted();
}

void callback_rpwc_gripper_cmd(const rpwc_msgs::RobotEeStateStamped::ConstPtr& msg)
{
	double position = msg->position.data;
	if(msg->position.data < 0.0)position = 0.0;
	else if (msg->position.data > 1.0) position = 1.0;

	double velocity = 0;
	double force = 0;

	position = (position *1.5) - 0.5;
	cmd_msg_.joint_names.resize(1);
  	cmd_msg_.joint_names[0] = "qbsoftclaw_deflection_virtual_joint";
	cmd_msg_.points.clear();
  	cmd_msg_.points.resize(1);
  	cmd_msg_.points[0].positions.push_back(position);
  	cmd_msg_.points[0].time_from_start = ros::Duration(0.3);

	if((lastCmdMsg_.position.data != position) || (lastCmdMsg_.velocity.data != velocity) || (lastCmdMsg_.force.data != force) ) pub_gripper_des_.publish(cmd_msg_);

	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();
}

bool callback_robot_curr_pose(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	lastCmdMsg_.header.stamp = ros::Time::now();
	res.state = lastCmdMsg_;
	return true;
}

void callback_gripper_feedback(const sensor_msgs::JointState::ConstPtr& msg)
{
	gripperFeedbackState_ = msg->position[3];
	if(!inizialized_)
	{
		inizialized_ = true;
		lastCmdMsg_.position.data = gripperFeedbackState_;
		if(lastCmdMsg_.position.data > 0.82) lastCmdMsg_.position.data = 1.0;
		lastCmdMsg_.velocity.data = 0;
		lastCmdMsg_.force.data = 0;
	}
}

bool callback_robot_curr_state(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	res.state.position.data = gripperFeedbackState_;
	return true;
}

void pub_for_recording_callback()
{
	ros::NodeHandle nhThread;
	double rate_10Hz = 10.0;
	ros::Rate r_10HZ(rate_10Hz);
	
	while(ros::ok())
	{
		pub_for_recording_.publish(lastCmdMsg_);
		ros::spinOnce();
		r_10HZ.sleep();
	}// end while()

	return;
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "softclaw_bridge_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
    spinner.start();

	//Subscriber
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
	ros::Subscriber sub_gripper_feedback = nh.subscribe("qbsoftclaw/control/joint_states", 1, &callback_gripper_feedback);
  	//Publisher
    pub_gripper_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("qbsoftclaw/control/qbsoftclaw_deflection_trajectory_controller/command", 1);
    pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
	//Service Server
	ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("rpwc_EE_cmd", &callback_server_rpwc_gripper_cmd);
  	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", &callback_robot_curr_pose);
	ros::ServiceServer server_robot_curr_state = nh.advertiseService("rpwc_robot_curr_state", &callback_robot_curr_state);
	
	eeCmdMove ee_cmd_move(nh);

	// fare un thread con pub for recording a 10 hz
	std::thread pubForRecordingThread(&pub_for_recording_callback);
	pubForRecordingThread.detach();

    ros::waitForShutdown();
	return 0;
}

