#/**
 * @file main_softhand_bridge.cpp
 * @brief RPWC bridge node for the qb SoftHand Research end-effector.
 *
 * This node translates normalized RPWC end-effector commands into qbHand
 * synergy-joint trajectory commands. It exposes the current gripper state,
 * periodically publishes the cached state for recording, and reports
 * hardware initialization status to the RPWC hardware manager.
 *
 */
#include <ros/ros.h>
#include <rpwc_msgs/robotEeCmdAction.h>
#include <rpwc_msgs/robotEeCmd.h>
#include <rpwc_msgs/checkHardwareStatus.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/robotEeState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <algorithm>
#include <cstddef>

/** @brief Publishers for RPWC recording and SoftHand trajectory commands. */
ros::Publisher pub_for_recording_, pub_gripper_des_;

/** @brief Last normalized RPWC command/state cached by the bridge. */
rpwc_msgs::RobotEeStateStamped lastCmdMsg_;

typedef actionlib::SimpleActionServer<rpwc_msgs::robotEeCmdAction> robotEeCmdAS;

/** @brief Latest raw SoftHand synergy-joint feedback. */
double gripperFeedbackState_ = 0.0;

/** @brief True after the first valid feedback message from the qb driver. */
bool initialized_ = false;

/** @brief Hardware check status exposed through check_hardware_status. */
int	hw_check_status_ = 0;  ///< 0 = CHECKING, 1 = END_CHECK.
bool hw_check_result_ = false;  ///< True when the hardware check completed successfully.
std::string hw_check_info_ = "";  ///< Human-readable hardware check details.

/**
 * @brief Internal helpers and constants for the SoftHand bridge.
 *
 * These helpers are kept in an anonymous namespace to limit symbol
 * visibility to this translation unit.
 */
namespace
{
/** @brief Name of the qbHand synergy joint used by the controller. */
const std::string kSynergyJointName = "qbhand_synergy_joint";

/** @brief Minimum normalized RPWC position. */
constexpr double kRpwcPositionMin = 0.0;
/** @brief Maximum normalized RPWC position. */
constexpr double kRpwcPositionMax = 1.0;
/** @brief Duration assigned to each trajectory command (seconds). */
constexpr double kTrajectoryDurationSec = 0.3;
/** @brief Fixed wait used by the action callback (seconds). */
constexpr double kActionCompletionWaitSec = 1.0;

/**
 * @brief Clamp a scalar value to an inclusive range.
 * @param value Input value.
 * @param min_value Lower bound.
 * @param max_value Upper bound.
 * @return The clamped value.
 */
double clamp_value(double value, double min_value, double max_value)
{
	return std::max(min_value, std::min(value, max_value));
}

/**
 * @brief Clamp a normalized RPWC position to the supported range.
 * @param position Requested normalized RPWC position.
 * @return Position clamped to [0, 1].
 */
double clamp_rpwc_position(double position)
{
	return clamp_value(position, kRpwcPositionMin, kRpwcPositionMax);
}

/**
 * @brief Read a joint position from a JointState message by joint name.
 * @param msg JointState feedback message.
 * @param joint_name Joint name to look up.
 * @param position Output joint position when found.
 * @return True when both the joint name and its position value are available.
 */
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
 * @brief Publish a qbHand trajectory command and update the cached RPWC state.
 * @param requested_position Requested normalized RPWC position.
 * @param force_publish If true, publish even when the command matches the cached state.
 * @return True when the command message has been prepared successfully.
 */
bool publish_softhand_command(double requested_position, bool force_publish)
{
	const double position = clamp_rpwc_position(requested_position);
	const double velocity = 0.0; // placeholder for RPWC interface
	const double force = 0.0;    // placeholder for RPWC interface

	trajectory_msgs::JointTrajectory cmd_msg;
	cmd_msg.joint_names.resize(1);
	cmd_msg.joint_names[0] = kSynergyJointName;
	cmd_msg.points.resize(1);
	cmd_msg.points[0].positions.push_back(position);
	cmd_msg.points[0].time_from_start = ros::Duration(kTrajectoryDurationSec);

	if(force_publish || (lastCmdMsg_.position.data != position) || (lastCmdMsg_.velocity.data != velocity) || (lastCmdMsg_.force.data != force))
	{
		pub_gripper_des_.publish(cmd_msg);
	}

	lastCmdMsg_.position.data = position;
	lastCmdMsg_.velocity.data = velocity;
	lastCmdMsg_.force.data = force;
	lastCmdMsg_.header.stamp = ros::Time::now();

	return true;
}
} // namespace

/**
 * @class eeCmdMove
 * @brief Action server wrapper for RPWC qbHand end-effector commands.
 *
 * Handles incoming RPWC `robotEeCmdAction` goals and translates them into
 * qbHand trajectory commands. The server runs in the node's context and
 * uses the same publish helpers as the service/callback APIs.
 */
class eeCmdMove
{
	protected:
		robotEeCmdAS as_;
		rpwc_msgs::robotEeCmdGoalConstPtr ee_goal_;
		rpwc_msgs::robotEeCmdResult ee_result_;

public:
	/**
	 * @brief Construct and start the qbHand action server.
	 * @param nh_ ROS node handle used to advertise the action server.
	 */
	eeCmdMove(ros::NodeHandle& nh_);

	/** @brief Destroy the action server wrapper. */
	~eeCmdMove();

	/** @brief Accept and execute a new RPWC end-effector action goal. */
	void goal_callback();

	/** @brief Handle action preemption requests. */
	void preempt_callback();
};

eeCmdMove::eeCmdMove(ros::NodeHandle& nh_) : as_(nh_, "rpwc_EE_action_cmd", false)
{
	as_.registerGoalCallback(boost::bind(&eeCmdMove::goal_callback, this));
	as_.registerPreemptCallback(boost::bind(&eeCmdMove::preempt_callback, this));
	as_.start();
	ROS_INFO_STREAM("eeCmdMove action server started.");
}

eeCmdMove::~eeCmdMove()
{
	ROS_INFO_STREAM("eeCmdMove removed.");
}

/**
 * @brief Service callback for RPWC qbHand position commands.
 * @param req Service request containing the desired normalized RPWC state.
 * @param res Service response, currently unused.
 * @return True when the command has been accepted for publication.
 */
bool callback_server_rpwc_gripper_cmd(rpwc_msgs::robotEeCmd::Request  &req, rpwc_msgs::robotEeCmd::Response &res)
{
	return publish_softhand_command(req.state.position.data, true);
}

void eeCmdMove::goal_callback()
{
	ee_goal_ = as_.acceptNewGoal();
	ROS_INFO_STREAM("Received goal with force: " << ee_goal_->state.force.data
					<< ", position: " << ee_goal_->state.position.data
					<< ", velocity: " << ee_goal_->state.velocity.data);

	if(!publish_softhand_command(ee_goal_->state.position.data, true))
	{
		ee_result_.success = false;
		ee_result_.msg = "Error during command execution.";
		as_.setAborted(ee_result_);
		return;
	}

	ros::Duration(kActionCompletionWaitSec).sleep();

	ee_result_.success = true;
	ee_result_.msg = "Execution completed successfully.";
	as_.setSucceeded(ee_result_);
}

void eeCmdMove::preempt_callback()
{
	ROS_WARN("Goal preempted");
	as_.setPreempted();
}

/**
 * @brief Accept and execute a new action goal.
 *
 * The callback accepts the goal, publishes the equivalent qbHand command,
 * waits a short fixed duration for execution, and reports success/failure.
 */
void callback_rpwc_gripper_cmd(const rpwc_msgs::RobotEeStateStamped::ConstPtr& msg)
{
	publish_softhand_command(msg->position.data, false);
}

/**
 * @brief Topic callback for RPWC qbHand position commands.
 * @param msg Desired normalized RPWC state.
 */
bool callback_robot_curr_pose(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	lastCmdMsg_.header.stamp = ros::Time::now();
	res.state = lastCmdMsg_;
	return true;
}

/**
 * @brief Return the last RPWC command sent to the qbHand.
 * @param req Service request, unused.
 * @param res Service response filled with the cached command state.
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
 * @brief Report qbHand hardware initialization status to the RPWC hardware manager.
 * @param req Service request, unused.
 * @param res Service response populated with hardware check status, result, and info.
 * @return Always true.
 */

void callback_gripper_feedback(const sensor_msgs::JointState::ConstPtr& msg)
{
	double position_feedback = 0.0;
	if(!get_joint_position(msg, kSynergyJointName, position_feedback))
	{
		hw_check_status_ = 1; // END_CHECK
		hw_check_result_ = false;
		hw_check_info_ = "Invalid qbhand joint_states feedback: missing position for " + kSynergyJointName + ".";
		ROS_WARN_STREAM_THROTTLE(5.0, hw_check_info_);
		return;
	}

	gripperFeedbackState_ = position_feedback;
	if(!initialized_)
	{
		initialized_ = true;
		lastCmdMsg_.position.data = clamp_rpwc_position(gripperFeedbackState_);
		if(lastCmdMsg_.position.data > 0.80) lastCmdMsg_.position.data = 1.0;
		lastCmdMsg_.velocity.data = 0;
		lastCmdMsg_.force.data = 0;
		hw_check_status_ = 1; // END_CHECK
		hw_check_result_ = true;
		hw_check_info_.clear();
	}
}

/**
 * @brief Update qbHand feedback and complete the startup hardware check.
 * @param msg JointState message published by the qbHand controller.
 */
bool callback_robot_curr_state(rpwc_msgs::robotEeState::Request  &req, rpwc_msgs::robotEeState::Response &res)
{
	res.state.position.data = initialized_ ? clamp_rpwc_position(gripperFeedbackState_) : lastCmdMsg_.position.data;
	res.state.velocity.data = lastCmdMsg_.velocity.data;
	res.state.force.data = lastCmdMsg_.force.data;
	return true;
}

/**
 * @brief Publish the cached RPWC state for recording.
 * @param event Timer event metadata, unused.
 */
void pub_for_recording_callback(const ros::TimerEvent& event)
{
	pub_for_recording_.publish(lastCmdMsg_);
}

/**
 * @brief Initialize the ROS node, interfaces, action server, and recording timer.
 * @param argc Argument count passed by ROS.
 * @param argv Argument values passed by ROS.
 * @return Process exit code.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "softhand_bridge_node");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
	spinner.start();

	// Subscribers
	ros::Subscriber sub_rpwc_gripper_cmd = nh.subscribe("rpwc_EE_cmd", 1, &callback_rpwc_gripper_cmd);
	ros::Subscriber sub_gripper_feedback = nh.subscribe("qbhand/control/joint_states", 1, &callback_gripper_feedback);
	// Publishers
	pub_gripper_des_ = nh.advertise<trajectory_msgs::JointTrajectory>("qbhand/control/qbhand_synergy_trajectory_controller/command", 1);
	pub_for_recording_ = nh.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
	// Services
	ros::ServiceServer server_check_hw_status = nh.advertiseService("check_hardware_status", &callback_check_hardware_status);
	ros::ServiceServer server_rpwc_gripper_cmd = nh.advertiseService("rpwc_EE_cmd", &callback_server_rpwc_gripper_cmd);
	ros::ServiceServer server_robot_curr_pose = nh.advertiseService("rpwc_robot_curr_pose", &callback_robot_curr_pose);
	ros::ServiceServer server_robot_curr_state = nh.advertiseService("rpwc_robot_curr_state", &callback_robot_curr_state);

	eeCmdMove ee_cmd_move(nh);

	ros::Timer pub_for_recording_timer = nh.createTimer(ros::Duration(0.1), &pub_for_recording_callback);

	ros::waitForShutdown();
	return 0;
}

