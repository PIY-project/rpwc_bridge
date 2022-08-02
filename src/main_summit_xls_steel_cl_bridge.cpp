#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "summit_xls_steel_cl_bridge.h"

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "summit_xls_steel_cl_bridge_node");
	summit_xls_steel_cl_bridge Obj;
  	
  	double rate_Hz = 20.0;
	ros::Rate rate_HZ(rate_Hz);


	while(ros::ok())
	{
		ros::spinOnce();
		rate_HZ.sleep();
	}// end while() 
	return 0;
}