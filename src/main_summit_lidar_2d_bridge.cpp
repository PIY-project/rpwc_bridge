#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "summit_lidar_2d_bridge.h"

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "summit_lidar_2d_bridge_node");
	summit_lidar_2d_bridge Obj;
  	double rate_100Hz = 100.0;
	ros::Rate r_100HZ(rate_100Hz);
	
	Obj.dt_ = 1.0/rate_100Hz;

    ROS_INFO("Summit lidarbridge node on. Ready to receive response");

	ros::spin();

	return 0;
}