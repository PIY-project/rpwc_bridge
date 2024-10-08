#include <ros/ros.h>
#include <rpwc_msgs/robotMobileBaseState.h>
#include <rpwc_msgs/RobotMobileBaseControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>


#include <eigen3/Eigen/Eigen>

# define PI 3.14159


class summit_xls_steel_cl_bridge
{
public:
	summit_xls_steel_cl_bridge();
	~summit_xls_steel_cl_bridge();


  double dt_;

private:
	ros::NodeHandle n_;

	// void callback_curr_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void callback_rpwc_control_poses(const rpwc_msgs::RobotMobileBaseControl::ConstPtr& msg);
	bool callback_robot_curr_pose(rpwc_msgs::robotMobileBaseState::Request  &req, rpwc_msgs::robotMobileBaseState::Response &res);
	geometry_msgs::PoseStamped pose_curr;

	ros::Time current_time, old_time;
	ros::Subscriber odom_sub_, sub_rpwc_control_poses_;
	ros::Publisher pub_poses_control_ ,pub_curr_pos_;
	ros::ServiceServer server_robot_curr_pose_;


};//End of class SubscribeAndPublish