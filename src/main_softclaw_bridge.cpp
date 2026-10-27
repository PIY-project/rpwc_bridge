/**
 * RPWC bridge for the qb SoftClaw. A stabilized early close is not proof that
 * an object has been grasped. The qb trajectory controller does not apply the
 * RPWC force and velocity command fields.
 */

#include <actionlib/server/simple_action_server.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <rpwc_bridge/softclaw_motion.h>
#include <rpwc_msgs/RobotEeStateStamped.h>
#include <rpwc_msgs/checkHardwareStatus.h>
#include <rpwc_msgs/robotEeCmd.h>
#include <rpwc_msgs/robotEeCmdAction.h>
#include <rpwc_msgs/robotEeState.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>

namespace
{
constexpr double kDeflectionMin = -0.5;
constexpr double kDeflectionMax = 1.0;
constexpr double kTrajectoryDurationSec = 0.3;
constexpr double kDuplicateRepublishSec = 1.0;
const char kDeflectionJoint[] = "qbsoftclaw_deflection_virtual_joint";

using Clock = rpwc_bridge::SoftClawMotion::Clock;
using TimePoint = Clock::time_point;
using ActionServer = actionlib::SimpleActionServer<rpwc_msgs::robotEeCmdAction>;

double clampPosition(double position)
{
  return std::max(0.0, std::min(1.0, position));
}

double toDeflection(double position)
{
  return kDeflectionMin + clampPosition(position) * (kDeflectionMax - kDeflectionMin);
}

double toNormalized(double deflection)
{
  return clampPosition((deflection - kDeflectionMin) / (kDeflectionMax - kDeflectionMin));
}

double elapsedSeconds(TimePoint now, TimePoint then)
{
  return std::chrono::duration<double>(now - then).count();
}

struct BridgeConfig
{
  rpwc_bridge::SoftClawMotionConfig motion;
  double feedback_timeout_sec = 0.5;
  double startup_timeout_sec = 10.0;
  double evaluation_period_sec = 0.05;
};

double positiveParam(const ros::NodeHandle& nh, const std::string& name, double default_value)
{
  double value = default_value;
  nh.param(name, value, default_value);
  if (!std::isfinite(value) || value <= 0.0)
  {
    ROS_WARN_STREAM("Invalid ~" << name << "=" << value << "; using " << default_value);
    return default_value;
  }
  return value;
}

BridgeConfig loadConfig(const ros::NodeHandle& nh)
{
  BridgeConfig config;
  config.motion.position_tolerance = positiveParam(nh, "position_tolerance", 0.02);
  config.motion.target_stability_sec = positiveParam(nh, "target_stability_sec", 0.2);
  config.motion.early_stability_sec = positiveParam(nh, "early_stability_sec", 0.5);
  config.motion.stability_delta = positiveParam(nh, "stability_delta", 0.01);
  config.motion.min_close_progress = positiveParam(nh, "min_close_progress", 0.02);
  config.motion.motion_timeout_sec = positiveParam(nh, "motion_timeout_sec", 8.0);
  config.feedback_timeout_sec = positiveParam(nh, "feedback_timeout_sec", 0.5);
  config.startup_timeout_sec = positiveParam(nh, "startup_timeout_sec", 10.0);
  config.evaluation_period_sec = positiveParam(nh, "evaluation_period_sec", 0.05);
  return config;
}

bool jointPosition(const sensor_msgs::JointState::ConstPtr& msg, double& position)
{
  const auto it = std::find(msg->name.begin(), msg->name.end(), kDeflectionJoint);
  if (it == msg->name.end())
    return false;
  const std::size_t index = static_cast<std::size_t>(it - msg->name.begin());
  if (index >= msg->position.size() || !std::isfinite(msg->position[index]))
    return false;
  position = msg->position[index];
  return true;
}
}  // namespace

class SoftClawBridge
{
public:
  SoftClawBridge()
    : private_nh_("~"),
      config_(loadConfig(private_nh_)),
      motion_(config_.motion),
      action_(nh_, "rpwc_EE_action_cmd", false),
      started_at_(Clock::now())
  {
    command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
        "qbsoftclaw/control/qbsoftclaw_deflection_trajectory_controller/command", 10);
    recording_pub_ = nh_.advertise<rpwc_msgs::RobotEeStateStamped>("rpwc_recording_EE", 1);
    command_sub_ = nh_.subscribe("rpwc_EE_cmd", 1, &SoftClawBridge::onTopicCommand, this);
    feedback_sub_ = nh_.subscribe("qbsoftclaw/control/joint_states", 1,
                                  &SoftClawBridge::onFeedback, this);
    command_service_ = nh_.advertiseService("rpwc_EE_cmd", &SoftClawBridge::onServiceCommand, this);
    pose_service_ = nh_.advertiseService("rpwc_robot_curr_pose", &SoftClawBridge::onPose, this);
    state_service_ = nh_.advertiseService("rpwc_robot_curr_state", &SoftClawBridge::onState, this);
    hardware_service_ = nh_.advertiseService("check_hardware_status", &SoftClawBridge::onHardware, this);
    evaluation_timer_ = nh_.createWallTimer(ros::WallDuration(config_.evaluation_period_sec),
                                            &SoftClawBridge::onEvaluation, this);
    recording_timer_ = nh_.createWallTimer(ros::WallDuration(0.1),
                                           &SoftClawBridge::onRecording, this);
    action_.registerGoalCallback(boost::bind(&SoftClawBridge::onGoal, this));
    action_.registerPreemptCallback(boost::bind(&SoftClawBridge::onPreempt, this));
    action_.start();
    ROS_INFO("SoftClaw bridge started; early close means stable motion, not verified grasp");
  }

private:
  bool feedbackFresh(TimePoint now) const
  {
    return has_feedback_ && elapsedSeconds(now, feedback_at_) <= config_.feedback_timeout_sec;
  }

  void cacheCommand(double position)
  {
    has_command_ = true;
    last_command_.position.data = position;
    last_command_.velocity.data = 0.0;
    last_command_.force.data = 0.0;
    last_command_.header.stamp = ros::Time::now();
  }

  void publishTrajectory(double deflection)
  {
    trajectory_msgs::JointTrajectory message;
    message.joint_names.push_back(kDeflectionJoint);
    message.points.resize(1);
    message.points[0].positions.push_back(deflection);
    message.points[0].time_from_start = ros::Duration(kTrajectoryDurationSec);
    command_pub_.publish(message);
  }

  bool holdMeasuredPosition(TimePoint now)
  {
    if (!feedbackFresh(now))
      return false;
    publishTrajectory(std::max(kDeflectionMin, std::min(kDeflectionMax, feedback_deflection_)));
    has_published_ = true;
    last_published_at_ = now;
    cacheCommand(feedback_position_);
    return true;
  }

  void preemptActive(const std::string& reason)
  {
    if (!motion_.active())
      return;
    const bool held = holdMeasuredPosition(Clock::now());
    motion_.reset();
    rpwc_msgs::robotEeCmdResult result;
    result.success = false;
    result.msg = reason + (held ? "; holding measured position"
                                : "; feedback stale, hold not guaranteed");
    ROS_WARN_STREAM("SoftClaw action preempted: " << result.msg);
    if (action_.isActive())
      action_.setPreempted(result);
  }

  bool acceptCommand(double requested_position, bool action_command, bool force_publish,
                     std::string& info)
  {
    const auto now = Clock::now();
    if (!std::isfinite(requested_position))
    {
      info = "Position must be finite";
      return false;
    }
    if (!feedbackFresh(now))
    {
      info = "No recent valid SoftClaw feedback";
      return false;
    }

    const double target = clampPosition(requested_position);
    if (!action_command && !force_publish && motion_.active() &&
        motion_.target() == target)
    {
      info = "Topic command already active";
      return true;
    }
    if (!action_command && !force_publish && !motion_.active() && has_published_ &&
        last_command_.position.data == target &&
        elapsedSeconds(now, last_published_at_) < kDuplicateRepublishSec)
    {
      info = "Duplicate topic command ignored";
      return true;
    }

    if (motion_.active())
      preemptActive("Replaced by another command");

    publishTrajectory(toDeflection(target));
    has_published_ = true;
    last_published_at_ = Clock::now();
    cacheCommand(target);
    if (action_command)
      motion_.start(feedback_position_, target, Clock::now());
    info = "Command accepted and published; motion completion is not confirmed";
    return true;
  }

  void onGoal()
  {
    if (motion_.active())
      preemptActive("Replaced by a new action goal");
    const auto goal = action_.acceptNewGoal();
    if (!goal)
      return;
    // actionlib may have received a cancel for this goal before acceptNewGoal().
    if (action_.isPreemptRequested())
    {
      rpwc_msgs::robotEeCmdResult result;
      result.success = false;
      result.msg = "Action goal canceled before command publication";
      action_.setPreempted(result);
      return;
    }

    std::string info;
    if (!acceptCommand(goal->state.position.data, true, true, info))
    {
      rpwc_msgs::robotEeCmdResult result;
      result.success = false;
      result.msg = info;
      action_.setAborted(result);
    }
    else if (action_.isPreemptRequested())
    {
      preemptActive("Action goal canceled");
    }
  }

  void onPreempt()
  {
    if (motion_.active() && action_.isActive() && action_.isPreemptRequested())
      preemptActive("Action goal canceled");
  }

  void finishAction(bool success, const std::string& message)
  {
    motion_.reset();
    rpwc_msgs::robotEeCmdResult result;
    result.success = success;
    result.msg = message;
    if (success)
      ROS_INFO_STREAM("SoftClaw action completed: " << message);
    else
      ROS_WARN_STREAM("SoftClaw action failed: " << message);
    if (!action_.isActive())
      return;
    if (success)
      action_.setSucceeded(result);
    else
      action_.setAborted(result);
  }

  void onEvaluation(const ros::WallTimerEvent&)
  {
    if (!motion_.active())
      return;
    const auto now = Clock::now();
    if (!feedbackFresh(now))
    {
      finishAction(false, "Feedback lost or stale; hold not guaranteed");
      return;
    }

    switch (motion_.evaluate(now))
    {
      case rpwc_bridge::SoftClawMotion::Result::TARGET_REACHED:
        finishAction(true, "Target position reached and stable");
        break;
      case rpwc_bridge::SoftClawMotion::Result::EARLY_CLOSE:
        finishAction(true, "Closing stabilized before target; object grasp is not verified");
        break;
      case rpwc_bridge::SoftClawMotion::Result::NO_PROGRESS:
        holdMeasuredPosition(now);
        finishAction(false, "Closing timed out without sufficient progress");
        break;
      case rpwc_bridge::SoftClawMotion::Result::TIMEOUT:
        holdMeasuredPosition(now);
        finishAction(false, "Motion timed out before completing");
        break;
      case rpwc_bridge::SoftClawMotion::Result::PENDING:
        break;
    }
  }

  void onFeedback(const sensor_msgs::JointState::ConstPtr& message)
  {
    double deflection = 0.0;
    if (!jointPosition(message, deflection))
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "Invalid SoftClaw joint_states: missing or non-finite "
                                      << kDeflectionJoint);
      return;
    }

    const auto now = Clock::now();
    feedback_deflection_ = deflection;
    feedback_position_ = toNormalized(deflection);
    feedback_at_ = now;
    feedback_ros_at_ = ros::Time::now();
    has_feedback_ = true;
    if (!has_command_)
      cacheCommand(feedback_position_);
    motion_.observe(feedback_position_, now);
  }

  void onTopicCommand(const rpwc_msgs::RobotEeStateStamped::ConstPtr& message)
  {
    std::string info;
    if (!acceptCommand(message->position.data, false, false, info))
      ROS_WARN_STREAM_THROTTLE(5.0, "SoftClaw topic command rejected: " << info);
  }

  bool onServiceCommand(rpwc_msgs::robotEeCmd::Request& request,
                        rpwc_msgs::robotEeCmd::Response& response)
  {
    std::string info;
    response.success.data = acceptCommand(request.state.position.data, false, true, info);
    response.info.data = info;
    return true;
  }

  bool onPose(rpwc_msgs::robotEeState::Request&, rpwc_msgs::robotEeState::Response& response)
  {
    if (!has_command_)
      return false;
    response.state = last_command_;
    return true;
  }

  bool onState(rpwc_msgs::robotEeState::Request&, rpwc_msgs::robotEeState::Response& response)
  {
    if (!feedbackFresh(Clock::now()))
      return false;
    response.state.position.data = feedback_position_;
    response.state.velocity.data = 0.0;
    response.state.force.data = 0.0;
    response.state.header.stamp = feedback_ros_at_;
    return true;
  }

  bool onHardware(rpwc_msgs::checkHardwareStatus::Request&,
                  rpwc_msgs::checkHardwareStatus::Response& response)
  {
    const auto now = Clock::now();
    if (feedbackFresh(now))
    {
      response.status.data = 1;  // END_CHECK
      response.result.data = true;
      response.info.data.clear();
    }
    else if (!has_feedback_ && elapsedSeconds(now, started_at_) < config_.startup_timeout_sec)
    {
      response.status.data = 0;  // CHECKING
      response.result.data = false;
      response.info.data = "Waiting for valid SoftClaw feedback";
    }
    else
    {
      response.status.data = 1;  // END_CHECK
      response.result.data = false;
      response.info.data = has_feedback_ ? "SoftClaw feedback is stale"
                                         : "Timed out waiting for valid SoftClaw feedback";
    }
    return true;
  }

  void onRecording(const ros::WallTimerEvent&)
  {
    if (!has_command_)
      return;
    auto recording = last_command_;
    recording.header.stamp = ros::Time::now();
    recording_pub_.publish(recording);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  BridgeConfig config_;
  rpwc_bridge::SoftClawMotion motion_;
  ActionServer action_;
  ros::Publisher command_pub_;
  ros::Publisher recording_pub_;
  ros::Subscriber command_sub_;
  ros::Subscriber feedback_sub_;
  ros::ServiceServer command_service_;
  ros::ServiceServer pose_service_;
  ros::ServiceServer state_service_;
  ros::ServiceServer hardware_service_;
  ros::WallTimer evaluation_timer_;
  ros::WallTimer recording_timer_;
  const TimePoint started_at_;
  TimePoint feedback_at_;
  ros::Time feedback_ros_at_;
  bool has_feedback_ = false;
  bool has_command_ = false;
  bool has_published_ = false;
  TimePoint last_published_at_;
  double feedback_deflection_ = 0.0;
  double feedback_position_ = 0.0;
  rpwc_msgs::RobotEeStateStamped last_command_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "softclaw_bridge_node");
  SoftClawBridge bridge;
  // All callbacks share one queue and one worker; no callback blocks waiting for motion.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
