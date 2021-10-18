#include <ros/ros.h>
#include <ros/package.h>
#include <rpwc_bridge/task_list.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include <yaml-cpp/yaml.h>


class request_from_web
{
	public:
		request_from_web();
		~request_from_web();


	  double dt_;

	private:
		ros::NodeHandle n_;

		bool callback_task_list(rpwc_bridge::task_list::Request  &req, rpwc_bridge::task_list::Response &res);

		void read_directory(const std::string& name, std::vector<std::string>& v);


		ros::ServiceServer server_task_list_;

		std::string path_pack_;

		struct path_leaf_string
		{
		  std::string operator()(const boost::filesystem::directory_entry& entry) const
		  {
		    return entry.path().leaf().string();
		  }
		};


};//End of class SubscribeAndPublish