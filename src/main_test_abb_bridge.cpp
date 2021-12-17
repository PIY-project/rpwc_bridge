#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "test_abb_bridge.h"

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_abb_bridge_node");
	test_abb_bridge Obj;

	ros::spin();

	return 0;
}