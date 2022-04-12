#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "zivid_bridge.h"

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "zivid_bridge_node");
	zivid_bridge Obj;
  	double rate_100Hz = 100.0;
	ros::Rate r_100HZ(rate_100Hz);
	
	Obj.dt_ = 1.0/rate_100Hz;

    ROS_INFO("Zivid bridge node on. Ready to receive response");

	ros::spin();

	return 0;
}