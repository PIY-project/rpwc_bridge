#include <ros/ros.h>
#include <rpwc/robot_curr_pose.h>
#include <rpwc_bridge/set_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <controller_manager_msgs/SwitchController.h>
#include <abb_driver/srv_abb_controller.h>


#include <eigen3/Eigen/Eigen>



class abb_bridge
{
public:
	abb_bridge();
	~abb_bridge();


  double dt_;

private:
	ros::NodeHandle n_;

	void callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg);
	void callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg);
	bool callback_robot_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res);
	bool callback_switch_controller(rpwc_bridge::set_controller::Request  &req, rpwc_bridge::set_controller::Response &res);


	ros::Subscriber sub_curr_pos_, sub_rpwc_pose_des_;
	ros::Publisher pub_pos_des_, pub_curr_pos_;
	ros::ServiceServer server_robot_curr_pose_, server_switch_controller_;
	ros::ServiceClient client_switch_controller_;

	geometry_msgs::Pose msg_pose_;


	Eigen::Quaterniond quat_base2EE_, quat_base2EE_old_;
	Eigen::Matrix4d T_base_2_EE_;
	bool first_quat_base_EE_;

};//End of class SubscribeAndPublish