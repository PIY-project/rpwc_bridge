# rpwc_bridge
rpwc_bridge

## SoftClaw bridge

`softclaw_bridge_node` accepts normalized RPWC positions in `[0, 1]` and sends
the corresponding deflection trajectory to the qb SoftClaw controller. Values
outside that interval are clamped; non-finite positions and commands without
recent valid joint feedback are rejected. RPWC `force` and `velocity` fields are
currently ignored by this controller.

The action succeeds when the measured position reaches its target and remains
there, or when a **closing** movement makes progress and then stabilizes before
the target. The latter means only that closing stopped early: position feedback
cannot distinguish an object from an obstruction and does not certify a grasp.
An opening must reach its target. On cancellation, the bridge sends a hold
trajectory to the most recently measured position if feedback is fresh. This is
a best-effort command, not a hardware emergency stop.

The command service reports whether the command was accepted and published;
the action reports its observed motion outcome. `rpwc_robot_curr_pose` and
`rpwc_recording_EE` expose the desired command for RPWC recording and replay;
`rpwc_robot_curr_state` exposes the measured position and fails while feedback
is stale. `check_hardware_status` waits for valid startup feedback and reports
stale feedback after initialization.

Private ROS parameters (seconds except normalized positions):

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `position_tolerance` | 0.02 | Maximum target position error |
| `target_stability_sec` | 0.2 | Time at target before action success |
| `early_stability_sec` | 0.5 | Stable window for early closing success |
| `stability_delta` | 0.01 | Maximum position range in that window |
| `min_close_progress` | 0.02 | Minimum advancement before early success |
| `motion_timeout_sec` | 8.0 | Maximum action duration |
| `feedback_timeout_sec` | 0.5 | Maximum age of valid feedback |
| `startup_timeout_sec` | 10.0 | Maximum wait for first valid feedback |
| `evaluation_period_sec` | 0.05 | Action evaluation period |

Calibrate the position thresholds and timeouts from empty and object-contact
movements on the actual SoftClaw before relying on early-closing outcomes.


POCO install step

wget https://pocoproject.org/releases/poco-1.8.1/poco-1.8.1.tar.gz

#Install dependences
sudo apt-get install openssl libssl-dev
sudo apt-get install libiodbc2 libiodbc2-dev
sudo apt-get install libmysqlclient-dev

cd poco-1.8.1
#sudo ./configure --omit=Data/ODBC,Data/MySQL --no-tests --no-samples --shared
cd build 
cmake-gui ..
sudo make -j8 
sudo make install 
