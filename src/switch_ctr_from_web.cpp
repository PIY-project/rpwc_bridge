#include <switch_ctr_from_web.h>

switch_ctr_from_web::switch_ctr_from_web()
{
	service_arm_controller_web_ = n_.advertiseService("/rpwc_switch_arm_controller_web", &switch_ctr_from_web::callback_arm_controller_web, this);

}

switch_ctr_from_web::~switch_ctr_from_web()
{

}

bool switch_ctr_from_web::callback_arm_controller_web(rpwc_bridge::set_controller_web::Request  &req, rpwc_bridge::set_controller_web::Response &res)
{
    rpwc_bridge::set_controller SetController;
    SetController.request.controller_name.data = req.controller_name.data;

    std::string controller_ns = req.ns.data;
    std::string topic_name;
    topic_name = "/" + controller_ns + "/rpwc_switch_arm_controller";
    ros::ServiceClient client_switch_controller_ = n_.serviceClient<rpwc_bridge::set_controller>(topic_name);
    if (!client_switch_controller_.call(SetController)) ROS_ERROR("Failed to call service client_switch_controller in main_switch_ctr_from_web");

    res.answer.data = true;
    return true;
}