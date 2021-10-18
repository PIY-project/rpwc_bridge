#include <ros/ros.h>
#include <request_from_web.h>







int main(int argc, char** argv){
    ros::init(argc, argv, "node_request_from_web");
    request_from_web Obj;
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