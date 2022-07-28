#include <ros/ros.h>
#include <rpwc_msgs/robotMobileBaseState.h>
#include <geometry_msgs/Pose.h>
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

	void callback_curr_pose(const nav_msgs::Odometry::ConstPtr& msg);
	void callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg);
	bool callback_robot_curr_pose(rpwc_msgs::robotMobileBaseState::Request  &req, rpwc_msgs::robotMobileBaseState::Response &res);

	nav_msgs::Odometry odom_curr;
	ros::Subscriber sub_curr_pos_, sub_rpwc_pose_des_;
	ros::Publisher pub_pos_des_ ,pub_curr_pos_;
	ros::ServiceServer server_robot_curr_pose_;


};//End of class SubscribeAndPublish