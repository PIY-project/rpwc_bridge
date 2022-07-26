#include <summit_xls_steel_cl_bridge.h>

summit_xls_steel_cl_bridge::summit_xls_steel_cl_bridge()
{
	//Subscriber
	sub_curr_pos_ = n_.subscribe("robot/robotnik_base_control/odom", 1, &summit_xls_steel_cl_bridge::callback_curr_pose, this);
	sub_rpwc_pose_des_ = n_.subscribe("rpwc_pose_des", 1, &summit_xls_steel_cl_bridge::callback_rpwc_pose_des, this);
	//Publisher
    pub_pos_des_ = n_.advertise<geometry_msgs::Pose2D>("pose_des", 1);
    pub_vel_des_ = n_.advertise<geometry_msgs::Twist>("vel_des", 1);
    pub_curr_pos_ = n_.advertise<rpwc_msgs::Pose2DStamped>("rpwc_robot_curr_pose", 1);
	//Service Server
  	server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &summit_xls_steel_cl_bridge::callback_robot_curr_pose, this);
  	
}

summit_xls_steel_cl_bridge::~summit_xls_steel_cl_bridge()
{

}

double summit_xls_steel_cl_bridge::UnwrapAngle(double prev_angle, double new_angle)
{
    return prev_angle - AngleDiff(new_angle, prev_angle);
}

double summit_xls_steel_cl_bridge::AngleDiff(double angle_a, double angle_b) {
    double diff = std::fmod(angle_b - angle_a + PI, 2*PI);
    if (diff < 0)
        diff += 2*PI;

    return diff - PI;
}


void summit_xls_steel_cl_bridge::callback_curr_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
	//come gestisco l'offset? lo deve fare rpwc

    Eigen::Quaterniond q_tmp;
    q_tmp.x() = msg->pose.pose.orientation.x;
    q_tmp.y() = msg->pose.pose.orientation.y;
    q_tmp.z() = msg->pose.pose.orientation.z;
    q_tmp.w() = msg->pose.pose.orientation.w;
    Eigen::Vector3d euler = q_tmp.normalized().toRotationMatrix().eulerAngles(0, 1, 2);

	yaw_curr = euler[2];
	x_curr = msg->pose.pose.position.x;
	y_curr = msg->pose.pose.position.y;
	rpwc_msgs::Pose2DStamped msg_pose;
	msg_pose.pose2D.theta = yaw_curr;
	msg_pose.pose2D.x = x_curr;
	msg_pose.pose2D.y = y_curr;
	msg_pose.header.stamp = ros::Time::now();
	pub_curr_pos_.publish(msg_pose);
}

void summit_xls_steel_cl_bridge::callback_rpwc_pose_des(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	double k_p = 1.0;  // gain for position controller
    double k_th = 1.0;  // gain for orientation controller

	geometry_msgs::Pose2D send_pose;
	send_pose.x = msg->x;
	send_pose.y = msg->y;
	send_pose.theta = msg->theta;
	

// Compute the commanded linear velocity to send to the robot
    Eigen::Vector2d pos_diff((msg->x - x_curr), (msg->y - y_curr));
    double err_pos = pos_diff.norm();
    double v_cmd = k_p * err_pos;

  	// Compute orientation error with LOS angle as theta_des 
    double theta_aux = atan2(pos_diff(1), pos_diff(0));
    double theta_des = 0.0;
    theta_des = UnwrapAngle(theta_des, theta_aux);
    double err_theta = theta_des - msg->theta;  // orientation error with LOS angle
    double omega_cmd = k_th * err_theta;

    geometry_msgs::Twist send_vel;

    send_vel.linear.x = v_cmd;
    send_vel.angular.z = omega_cmd;


	pub_pos_des_.publish(send_pose);
	pub_vel_des_.publish(send_vel);
}

bool summit_xls_steel_cl_bridge::callback_robot_curr_pose(rpwc_msgs::robotMobileBaseState::Request  &req, rpwc_msgs::robotMobileBaseState::Response &res)
{
	rpwc_msgs::Pose2DStamped robot_curr_pose;
	robot_curr_pose.pose2D.x = x_curr;
	robot_curr_pose.pose2D.y = y_curr;
	robot_curr_pose.pose2D.theta = yaw_curr;
	robot_curr_pose.header.stamp = ros::Time::now();

	res.pose2D = robot_curr_pose;
	return true;
}


