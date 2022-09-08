
#include <ros/ros.h>

#include <rpwc_msgs/LaserScanReq.h>
#include <sensor_msgs/LaserScan.h>


sensor_msgs::LaserScan scan_;

bool callbackServerLidar2D(rpwc_msgs::LaserScanReq::Request  &req, rpwc_msgs::LaserScanReq::Response &res)
{
    res.scan = scan_;
	return true;
}

void callback_scan(const sensor_msgs::LaserScanPtr& msg)
{
  scan_ = *msg;
}


int main(int argc, char** argv)
{
	//----------------------------------------------------------
	// Preparations
	//----------------------------------------------------------

	// Initialize the node.
	ros::init(argc, argv, "lidar_2D_bridge_node");
	ros::NodeHandle nh;
	ros::Rate rate(5);

	ros::ServiceServer serverLidar2D = nh.advertiseService("lidar2D", callbackServerLidar2D); 

    ros::Subscriber sub_scan = nh.subscribe("/robot/merged_laser/scan", 1, &callback_scan);
	
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
