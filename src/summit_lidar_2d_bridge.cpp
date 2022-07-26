#include <summit_lidar_2d_bridge.h>


summit_lidar_2d_bridge::summit_lidar_2d_bridge()
{
  ros::NodeHandle n;

  ROS_INFO("Starting summit_lidar_2d_bridge.cpp");

  // Subscibers
  scan_sub_             = n.subscribe("/robot/merged_laser/scan", 1, &summit_lidar_2d_bridge::on_scan, this);
  // Publishers
  scan_pub_ 		        = n.advertise<sensor_msgs::LaserScan>("/lidar_2d", 1);
  // Servers
  server_lidar_2d_capture_  = n.advertiseService("lidar_2d", &summit_lidar_2d_bridge::callback_lidar_2d_data, this);
  // Clients
}

summit_lidar_2d_bridge::~summit_lidar_2d_bridge(){}


void summit_lidar_2d_bridge::on_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("LaserScan received");
  msg_ls_  = *msg;
  scan_pub_.publish(msg_ls_);
}

bool summit_lidar_2d_bridge::callback_lidar_2d_data(rpwc_msgs::LaserScanReq::Request &req, rpwc_msgs::LaserScanReq::Response &res)
{
  
  res.data.scan = msg_ls_;

  return true;
}
