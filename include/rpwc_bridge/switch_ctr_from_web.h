#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rpwc_bridge/set_controller_web.h>
#include <rpwc_bridge/set_controller.h>

class switch_ctr_from_web
{
public:
	switch_ctr_from_web();
	~switch_ctr_from_web();


  double dt_;

private:
	ros::NodeHandle n_;

	bool callback_arm_controller_web(rpwc_bridge::set_controller_web::Request  &req, rpwc_bridge::set_controller_web::Response &res);


	ros::ServiceServer service_arm_controller_web_;

};//End of class SubscribeAndPublish