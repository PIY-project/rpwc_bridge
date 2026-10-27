#!/usr/bin/env python3
"""ROS integration test with synthetic SoftClaw joint feedback."""

import math
import threading
import time
import unittest

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from rpwc_msgs.msg import RobotEeStateStamped, robotEeCmdAction, robotEeCmdGoal
from rpwc_msgs.srv import checkHardwareStatus, robotEeCmd, robotEeState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


NS = "/softclaw_bridge_test/"
JOINT = "qbsoftclaw_deflection_virtual_joint"


def until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if predicate():
            return True
        time.sleep(0.02)
    return False


class SoftClawBridgeIntegration(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.position = 0.0
        self.feedback_mode = "invalid"
        self.commands = []
        self.stop = threading.Event()
        self.feedback_pub = rospy.Publisher(
            NS + "qbsoftclaw/control/joint_states", JointState, queue_size=20
        )
        self.topic_pub = rospy.Publisher(NS + "rpwc_EE_cmd", RobotEeStateStamped,
                                         queue_size=10)
        self.command_sub = rospy.Subscriber(
            NS + "qbsoftclaw/control/qbsoftclaw_deflection_trajectory_controller/command",
            JointTrajectory,
            self.on_command,
            queue_size=20,
        )
        self.feedback_thread = threading.Thread(target=self.publish_feedback)
        self.feedback_thread.daemon = True
        self.feedback_thread.start()
        rospy.wait_for_service(NS + "check_hardware_status", timeout=5.0)
        rospy.wait_for_service(NS + "rpwc_EE_cmd", timeout=5.0)
        rospy.wait_for_service(NS + "rpwc_robot_curr_pose", timeout=5.0)
        rospy.wait_for_service(NS + "rpwc_robot_curr_state", timeout=5.0)
        self.hardware = rospy.ServiceProxy(NS + "check_hardware_status", checkHardwareStatus)
        self.command = rospy.ServiceProxy(NS + "rpwc_EE_cmd", robotEeCmd)
        self.pose = rospy.ServiceProxy(NS + "rpwc_robot_curr_pose", robotEeState)
        self.state = rospy.ServiceProxy(NS + "rpwc_robot_curr_state", robotEeState)
        self.client = actionlib.SimpleActionClient(NS + "rpwc_EE_action_cmd", robotEeCmdAction)
        self.second_client = actionlib.SimpleActionClient(
            NS + "rpwc_EE_action_cmd", robotEeCmdAction
        )
        self.assertTrue(self.client.wait_for_server(rospy.Duration(5.0)))
        self.assertTrue(self.second_client.wait_for_server(rospy.Duration(5.0)))
        self.assertTrue(until(lambda: self.feedback_pub.get_num_connections() > 0))
        self.assertTrue(until(lambda: self.topic_pub.get_num_connections() > 0))

    def tearDown(self):
        self.stop.set()
        self.feedback_thread.join(timeout=1.0)
        self.feedback_pub.unregister()
        self.topic_pub.unregister()
        self.command_sub.unregister()

    def publish_feedback(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            with self.lock:
                position = self.position
                mode = self.feedback_mode
            if mode != "off":
                message = JointState()
                message.header.stamp = rospy.Time.now()
                message.name = [JOINT if mode != "missing" else "other_joint"]
                message.position = [float("nan") if mode == "invalid" else
                                    -0.5 + 1.5 * position]
                self.feedback_pub.publish(message)
            time.sleep(0.02)

    def on_command(self, message):
        with self.lock:
            self.commands.append(message.points[0].positions[0])

    def set_feedback(self, position=None, mode=None):
        with self.lock:
            if position is not None:
                self.position = position
            if mode is not None:
                self.feedback_mode = mode

    def goal(self, position):
        goal = robotEeCmdGoal()
        goal.state.position.data = position
        self.client.send_goal(goal)

    def test_bridge(self):
        # Malformed startup messages must not fail initialization immediately.
        self.assertTrue(until(lambda: self.hardware().status.data == 0))
        self.set_feedback(mode="missing")
        time.sleep(0.1)
        self.assertEqual(self.hardware().status.data, 0)
        self.set_feedback(mode="valid")
        self.assertTrue(until(lambda: self.hardware().result.data))

        invalid = self.command(state=self._request_state(float("nan")))
        self.assertFalse(invalid.success.data)
        self.assertIn("finite", invalid.info.data)

        # A full-close request may stop at an object before position 1.
        self.goal(1.0)
        for position in (0.1, 0.3, 0.5):
            self.set_feedback(position=position)
            time.sleep(0.08)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.SUCCEEDED)
        self.assertIn("before target", self.client.get_result().msg)
        self.assertAlmostEqual(self.pose().state.position.data, 1.0)
        self.assertAlmostEqual(self.state().state.position.data, 0.5, places=2)

        # An opening that stops before its target must fail.
        self.goal(0.0)
        self.set_feedback(position=0.3)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.ABORTED)

        # Cancellation commands the current measured position, not the old goal.
        before = self._command_count()
        self.goal(1.0)
        self.assertTrue(until(lambda: self._new_command_is(1.0, before)))
        self.set_feedback(position=0.4)
        time.sleep(0.08)
        self.client.cancel_goal()
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)
        self.assertTrue(until(lambda: self._last_command_is(0.1)))

        # A service command preempts an action and reports publication separately.
        before = self._command_count()
        self.goal(1.0)
        self.assertTrue(until(lambda: self._new_command_is(1.0, before)))
        accepted = self.command(state=self._request_state(0.0))
        self.assertTrue(accepted.success.data)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)
        self.assertTrue(until(lambda: self._last_command_is(-0.5)))

        # The topic also supersedes the active action.
        before = self._command_count()
        self.goal(1.0)
        self.assertTrue(until(lambda: self._new_command_is(1.0, before)))
        topic_command = self._request_state(0.0)
        self.topic_pub.publish(topic_command)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)

        # A second action goal owns the final result; the old one stays preempted.
        before = self._command_count()
        self.goal(1.0)
        self.assertTrue(until(lambda: self._new_command_is(1.0, before)))
        replacement = robotEeCmdGoal()
        replacement.state.position.data = 0.0
        self.second_client.send_goal(replacement)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)
        self.set_feedback(position=0.0)
        self.assertTrue(self.second_client.wait_for_result(rospy.Duration(2.0)))
        self.assertEqual(self.second_client.get_state(), GoalStatus.SUCCEEDED)

        # Feedback loss while an action is running must abort that action.
        before = self._command_count()
        self.goal(1.0)
        self.assertTrue(until(lambda: self._new_command_is(1.0, before)))
        self.set_feedback(mode="off")
        self.assertTrue(self.client.wait_for_result(rospy.Duration(2.0)))
        self.assertEqual(self.client.get_state(), GoalStatus.ABORTED)
        self.set_feedback(mode="valid")
        self.assertTrue(until(lambda: self.hardware().result.data))

        # A stopped feedback stream must prevent command acceptance.
        self.set_feedback(mode="off")
        time.sleep(0.4)
        stale = self.command(state=self._request_state(1.0))
        self.assertFalse(stale.success.data)
        self.assertFalse(self.hardware().result.data)

    @staticmethod
    def _request_state(position):
        state = RobotEeStateStamped()
        state.position.data = position
        return state

    def _last_command_is(self, expected):
        with self.lock:
            return bool(self.commands) and math.isclose(
                self.commands[-1], expected, abs_tol=0.01
            )

    def _command_count(self):
        with self.lock:
            return len(self.commands)

    def _new_command_is(self, expected, previous_count):
        with self.lock:
            return len(self.commands) > previous_count and math.isclose(
                self.commands[-1], expected, abs_tol=0.01
            )


if __name__ == "__main__":
    rospy.init_node("softclaw_bridge_integration_test")
    rostest.rosrun("rpwc_bridge", "softclaw_bridge_integration", SoftClawBridgeIntegration)
