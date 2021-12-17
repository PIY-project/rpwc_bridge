#include <ros/ros.h>
#include <rpwc/robot_curr_pose.h>
#include <rpwc_bridge/set_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <controller_manager_msgs/SwitchController.h>

#include <eigen3/Eigen/Eigen>



class arm_base_bridge
{
	public:
		arm_base_bridge();
		~arm_base_bridge();

		ros::NodeHandle n_;

		// void callback_curr_pose(const franka_msgs::FrankaState::ConstPtr& msg);
		void callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg);
		bool callback_robot_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res);


		ros::Subscriber sub_rpwc_pose_des_;
		ros::Publisher pub_pose_des_, pub_rpwc_curr_pose_;
		ros::ServiceServer server_robot_curr_pose_;

		Eigen::Quaterniond quat_base_2_lastlink_, quat_base_2_lastlink_old_;
		Eigen::Matrix4d T_base_2_lastlink_;
		bool first_quat_base_2_lastlink_;
		std::string controller_name_, old_controller_name_;
		std::vector<std::string> vec_ctr_name_;

	private:

};//End of class SubscribeAndPublish