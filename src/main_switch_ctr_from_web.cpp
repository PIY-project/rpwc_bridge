#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rpwc_bridge/set_controller_web.h>
#include <rpwc_bridge/set_controller.h>
#include <switch_ctr_from_web.h>







int main(int argc, char** argv){
    ros::init(argc, argv, "node_switch_ctr");
    switch_ctr_from_web Obj;
    double rate_10Hz = 10.0;
    ros::Rate r_10HZ(rate_10Hz);

    
    Obj.dt_ = 1.0/rate_10Hz;
    // ros::spin();
    while(ros::ok())
    {
        ros::spinOnce();
        r_10HZ.sleep();
        
    }// end while()
    return 0;
}