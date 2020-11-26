#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "ur_bridge.h"

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_bridge_node");
	ur_bridge Obj;
  	double rate_100Hz = 100.0;
	ros::Rate r_100HZ(rate_100Hz);

	
	Obj.dt_ = 1.0/rate_100Hz;

	while(ros::ok())
	{
		ros::spinOnce();
		r_100HZ.sleep();
	}// end while()
	return 0;
}