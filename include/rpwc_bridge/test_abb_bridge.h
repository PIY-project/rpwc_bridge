#include <ros/ros.h>
#include <arm_base_bridge.h>

#include <rpwc/robot_curr_pose.h>
#include <rpwc_bridge/set_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <controller_manager_msgs/SwitchController.h>
#include <abb_driver/srv_abb_controller.h>


#include <eigen3/Eigen/Eigen>



class test_abb_bridge : private arm_base_bridge
{
	public:
		test_abb_bridge();
		~test_abb_bridge();
	private:
		// ros::NodeHandle n_;

		void callback_curr_pose(const geometry_msgs::Pose::ConstPtr& msg);
		bool callback_switch_controller(rpwc_bridge::set_controller::Request  &req, rpwc_bridge::set_controller::Response &res);


		ros::Subscriber sub_curr_pos_;
		ros::ServiceServer server_switch_controller_;
		ros::ServiceClient client_switch_controller_;
};//End of class SubscribeAndPublish