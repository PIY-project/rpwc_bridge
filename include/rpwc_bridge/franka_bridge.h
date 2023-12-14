#include <ros/ros.h>
// #include <franka_msgs/FrankaState.h>
#include <rpwc_msgs/robotArmState.h>
#include <rpwc_msgs/setController.h>
#include <rpwc_bridge/set_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <controller_manager_msgs/SwitchController.h>


#include <eigen3/Eigen/Eigen>



class franka_bridge
{
public:
	franka_bridge();
	~franka_bridge();


  double dt_;

private:
	ros::NodeHandle n_;

	void callback_curr_pose(const  geometry_msgs::PoseStamped::ConstPtr& msg);
	void callback_curr_pose_gravity(const  geometry_msgs::PoseStamped::ConstPtr& msg);
	
	void callback_equi_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void callback_rpwc_pose_des(const geometry_msgs::PoseStamped::ConstPtr& msg);
	bool callback_robot_curr_pose(rpwc_msgs::robotArmState::Request  &req, rpwc_msgs::robotArmState::Response &res);
	bool callback_controller(rpwc_msgs::setController::Request  &req, rpwc_msgs::setController::Response &res);
	
	//bool callback_switch_controller(rpwc_bridge::set_controller::Request  &req, rpwc_bridge::set_controller::Response &res);



	ros::Subscriber sub_curr_pos_, sub_rpwc_pose_des_, sub_equi_pos_, sub_curr_pos_gravity_;
	ros::Publisher pub_pos_des_, pub_curr_pos_, pub_rec_pos_;
	ros::ServiceServer server_robot_curr_pose_, server_switch_controller_, srv_controller_;
	ros::ServiceClient client_switch_controller_;

	Eigen::Quaterniond quat_base2EE_, quat_base2EE_old_, quat_rec_, quat_rec_old_, quat_base2EE_gravity_, quat_base2EE_gravity_old_;
	bool first_quat_base_EE_, first_rec_, first_quat_base_EE_gravity_;
	std::string controller_name_, old_controller_name_;
	geometry_msgs::PoseStamped msg_pose_, msg_pose_gravity_;

};//End of class SubscribeAndPublish