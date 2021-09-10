#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "abb_bridge.h"

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "abb_bridge_node");
	abb_bridge Obj;
  	double rate_250Hz = 250.0;
	ros::Rate r_250HZ(rate_250Hz);

	
	Obj.dt_ = 1.0/rate_250Hz;

	while(ros::ok())
	{
		ros::spinOnce();
		r_250HZ.sleep();
	}// end while()
	return 0;
}