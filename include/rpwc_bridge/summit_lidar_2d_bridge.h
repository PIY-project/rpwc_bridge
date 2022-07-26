#include <ros/ros.h>

#include <rpwc_msgs/LaserScanReq.h>

#include <sensor_msgs/LaserScan.h>


class summit_lidar_2d_bridge
{
public:
	summit_lidar_2d_bridge();
	~summit_lidar_2d_bridge();

  	double dt_;

private:
	void on_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    bool callback_lidar_2d_data(rpwc_msgs::LaserScanReq::Request &req, rpwc_msgs::LaserScanReq::Response &res);

	ros::Subscriber 				scan_sub_;
	ros::Publisher 					scan_pub_;
	ros::ServiceServer 				server_lidar_2d_capture_;

	sensor_msgs::LaserScan		msg_ls_;

};