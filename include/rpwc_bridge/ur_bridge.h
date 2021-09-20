#include <ros/ros.h>
#include <rpwc/robot_curr_pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>


#include <eigen3/Eigen/Eigen>



class ur_bridge
{
public:
	ur_bridge();
	~ur_bridge();


  double dt_;

private:
	ros::NodeHandle n_;

	void callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg);
	void callback_rpwc_pose_des(const geometry_msgs::Pose::ConstPtr& msg);
	bool callback_robot_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res);
	void callback_rpwc_pass_obs(const geometry_msgs::Pose::ConstPtr& msg);
	bool callback_pass_obs_curr_pose(rpwc::robot_curr_pose::Request  &req, rpwc::robot_curr_pose::Response &res);



	ros::Subscriber sub_curr_pos_, sub_rpwc_pose_des_, sub_rpwc_pass_obs_;
	ros::Publisher pub_pos_des_, pub_curr_pos_, pub_pos_des_pass_obs_;
	ros::ServiceServer server_robot_curr_pose_, server_pass_obs_curr_pose_;

	Eigen::Quaterniond quat_base2EE_, quat_base2EE_old_, quat_base2EE_pass_obs_, quat_base2EE_pass_obs_old_;
	Eigen::Matrix4d T_base_2_EE_, T_base_2_EE_pass_obs_;
	bool first_quat_base_EE_, first_quat_base_EE_pass_obs_;

};//End of class SubscribeAndPublish