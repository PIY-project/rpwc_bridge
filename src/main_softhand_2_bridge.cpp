
/**
 * @file main_softhand_2_bridge.cpp
 * @brief RPWC bridge node for the qb SoftHand 2 end-effector.
 *
 * Translates normalized RPWC end-effector commands into qbHand2m
 * trajectory commands (manipulation + synergy channels), exposes the current
 * gripper state, periodically publishes the cached state for recording, and
 * reports hardware initialization status to the RPWC hardware manager.
 */
#include <ros/ros.h>
#include <rpwc_msgs/robotEeCmd.h>
#include <rpwc_msgs/checkHardwareStatus.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/robotEeState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <algorithm>
#include <cstddef>

trajectory_msgs::JointTrajectory send_hand_;

/** @brief Publishers for RPWC recording and qbHand2m trajectory commands. */
ros::Publisher pub_for_recording_, pub_hand_des_;

/** @brief Last normalized RPWC command/state cached by the bridge. */
rpwc_msgs::RobotEeStateStamped lastCmdMsg_;

/** @brief Latest raw qbHand2m joint feedback. */
double gripperManipulationState_ = 0.0;
double gripperSynergyState_ = 0.0;

/** @brief True after the first valid feedback message from the qb driver. */
bool initialized_ = false;

/** @brief Hardware check status exposed through check_hardware_status. */
int	hw_check_status_ = 0;  ///< 0 = CHECKING, 1 = END_CHECK.
bool hw_check_result_ = false;  ///< True when the hardware check completed successfully.
std::string hw_check_info_ = "";  ///< Human-readable hardware check details.

namespace
{
const std::string kManipJointName = "qbhand2m_manipulation_joint";
const std::string kSynergyJointName = "qbhand2m_synergy_joint";
constexpr double kRpwcPositionMin = 0.0;
constexpr double kRpwcPositionMax = 1.0;
constexpr double kTrajectoryDurationSec = 0.3;

double clamp_value(double value, double min_value, double max_value)
{
	return std::max(min_value, std::min(value, max_value));
}

double clamp_rpwc_position(double position)
{
	return clamp_value(position, kRpwcPositionMin, kRpwcPositionMax);
}

bool get_joint_position(const sensor_msgs::JointState::ConstPtr& msg, const std::string& joint_name, double& position)
{
	const auto joint_it = std::find(msg->name.begin(), msg->name.end(), joint_name);
	if(joint_it == msg->name.end())
	{
		return false;
	}

	const std::size_t joint_index = static_cast<std::size_t>(joint_it - msg->name.begin());
	if(joint_index >= msg->position.size())
	{
		return false;
	}

	position = msg->position[joint_index];
	return true;
}

/**
 * @brief Publish a qbHand2m trajectory command and update the cached RPWC state.
 * @param manip Requested manipulation channel value (normalized [0,1]).
 * @param synergy Requested synergy channel value (normalized [0,1]).
 * @param force_publish If true, publish even when the command matches cached state.
 * @return True when the command message has been prepared successfully.
 */
bool publish_hand2m_command(double manip, double synergy, bool force_publish)
{
	const double manip_v = clamp_rpwc_position(manip);
	const double synergy_v = clamp_rpwc_position(synergy);
	const double velocity = 0.0;
	const double force = 0.0;

	trajectory_msgs::JointTrajectory cmd_msg;
	cmd_msg.joint_names.resize(2);
	cmd_msg.joint_names[0] = kManipJointName;
	cmd_msg.joint_names[1] = kSynergyJointName;
	cmd_msg.points.resize(1);
	cmd_msg.points[0].positions.push_back(manip_v);
	cmd_msg.points[0].positions.push_back(synergy_v);
	cmd_msg.points[0].time_from_start = ros::Duration(kTrajectoryDurationSec);

	if(force_publish || (lastCmdMsg_.position.data != manip_v) || (lastCmdMsg_.velocity.data != synergy_v) || (lastCmdMsg_.force.data != force))
	{
		pub_hand_des_.publish(cmd_msg);
	}

	lastCmdMsg_.position.data = manip_v;
	lastCmdMsg_.velocity.data = synergy_v;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();

	return true;
}
} // namespace

/**
 * @brief Service callback for RPWC qbHand2m position commands.
 * @param req Service request containing the desired normalized RPWC state.
 * @param res Service response, currently unused.
 * @return True when the command has been accepted for publication.
 */
bool callback_rpwc_gripper_single_cmd(rpwc_msgs::robotEeCmd::Request  &req, rpwc_msgs::robotEeCmd::Response &res)
{
	double position = clamp_rpwc_position(req.state.position.data);
	double velocity = clamp_rpwc_position(req.state.velocity.data);
	double force = 0.0;

	return publish_hand2m_command(position, velocity, true);
}

/**
 * @brief Topic callback for RPWC qbHand2m position commands.
 * @param msg Desired normalized RPWC state.
 */
void callback_rpwc_gripper_cmd(const rpwc_msgs::RobotEeStateStamped::ConstPtr& msg)
{
	double position = clamp_rpwc_position(msg->position.data);
	double velocity = clamp_rpwc_position(msg->velocity.data);

	publish_hand2m_command(position, velocity, false);
	pub_for_recording_.publish(lastCmdMsg_);
}

/**
 * @brief Return the last RPWC command sent to the qbHand2m.
 * @param req Service request, unused.
 * @param res Service response filled with the cached command state.
 * @return Always true.
 */
bool callback_robot_curr_pose(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	lastCmdMsg_.header.stamp = ros::Time::now();
	res.state = lastCmdMsg_;
	return true;
}

/**
 * @brief Report qbHand2m hardware initialization status to the RPWC hardware manager.
 * @param req Service request, unused.
 * @param res Service response populated with hardware check status, result, and info.
 * @return Always true.
 */
bool callback_check_hardware_status(rpwc_msgs::checkHardwareStatus::Request& req, rpwc_msgs::checkHardwareStatus::Response& res)
{
	res.info.data   = hw_check_info_;
	res.result.data = hw_check_result_;
	res.status.data = hw_check_status_;
	return true;
}

/**
 * @brief Update qbHand2m feedback and complete the startup hardware check.
 * @param msg JointState message published by the qbHand2m controller.
 */
void callback_gripper_feedback(const sensor_msgs::JointState::ConstPtr& msg)
{
	double manip = 0.0;
	double synergy = 0.0;
	bool ok_manip = get_joint_position(msg, kManipJointName, manip);
	bool ok_synergy = get_joint_position(msg, kSynergyJointName, synergy);

	if(!ok_manip || !ok_synergy)
	{
		hw_check_status_ = 1; // END_CHECK
		hw_check_result_ = false;
		hw_check_info_ = "Invalid qbhand2m joint_states feedback: missing manipulation/synergy positions.";
		ROS_WARN_STREAM_THROTTLE(5.0, hw_check_info_);
		return;
	}

	gripperManipulationState_ = manip;
	gripperSynergyState_ = synergy;

	if(!initialized_)
	{
		initialized_ = true;
		lastCmdMsg_.position.data = clamp_rpwc_position(gripperManipulationState_);
		lastCmdMsg_.velocity.data = clamp_rpwc_position(gripperSynergyState_);
		lastCmdMsg_.force.data = 0;
		hw_check_status_ = 1; // END_CHECK
		hw_check_result_ = true;
		hw_check_info_.clear();
	}
}

/**
 * @brief Publish the cached RPWC state for recording.
 * @param event Timer event metadata, unused.
 */
void pub_for_recording_callback(const ros::TimerEvent& event)
{
	pub_for_recording_.publish(lastCmdMsg_);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "softhand_2_bridge_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
	spinner.start();

	send_hand_.joint_names.resize(2);
	send_hand_.joint_names[0] = kManipJointName;
	send_hand_.joint_names[1] = kSynergyJointName;

	lastCmdMsg_.position.data = 0.0;
	lastCmdMsg_.velocity.data = 0.0;
	lastCmdMsg_.force.data = 0.0;
	lastCmdMsg_.header.stamp = ros::Time::now();

	// Subscribers
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
	ros::Subscriber sub_gripper_feedback = nh.subscribe("qbhand2m/control/joint_states", 1, &callback_gripper_feedback);
	// Publishers
	pub_hand_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("qbhand2m/control/qbhand2m_synergies_trajectory_controller/command", 1);
	pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
	// Service Server
	ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("rpwc_EE_cmd", &callback_rpwc_gripper_single_cmd);
	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", &callback_robot_curr_pose);
	ros::ServiceServer server_check_hw_status = nh.advertiseService("check_hardware_status", &callback_check_hardware_status);

	ros::Timer pub_for_recording_timer = nh.createTimer(ros::Duration(0.1), &pub_for_recording_callback);

	ros::waitForShutdown();
	return 0;
}

