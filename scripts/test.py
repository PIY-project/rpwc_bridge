import time

import roslibpy

client = roslibpy.Ros(host='192.168.131.88', port=9090)
client.run()

talker = roslibpy.Topic(client, '/chatter', 'std_msgs/String')

while client.is_connected:
    talker.publish(roslibpy.Message({'data': 'Hello World!'}))
    print('Sending message...')
    time.sleep(1)

talker.unadvertise()

client.terminate()


# //Subscriber
	# odom_sub_ = n_.subscribe("/robot/robotnik_base_control/odom", 1, &summit_xls_steel_cl_bridge::odom_callback, this);
	# nuc sub_rpwc_control_poses_ = n_.subscribe("rpwc_pose_des", 1, &summit_xls_steel_cl_bridge::callback_rpwc_control_poses, this);
	
	# //Publisher
    # pub_poses_control_ = n_.advertise<rpwc_msgs::RobotMobileBaseControl>("poses_control", 1);
    # nuc pub_curr_pos_ = n_.advertise<geometry_msgs::PoseStamped>("rpwc_robot_curr_pose", 1);
	# //Service Server
  	# nuc server_robot_curr_pose_ = n_.advertiseService("rpwc_robot_curr_pose", &summit_xls_steel_cl_bridge::callback_robot_curr_pose, this);
	# current_time, old_time = ros::Time::now();