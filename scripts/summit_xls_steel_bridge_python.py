#!/usr/bin/env python

import rospy
from rpwc_msgs.msg import RobotMobileBaseControl
from geometry_msgs.msg import Twist
from rpwc_msgs.srv import robotMobileBaseState, robotMobileBaseStateResponse
from geometry_msgs.msg import PoseStamped


import roslibpy



class CustomNode:
    def __init__(self):
        rospy.init_node('custom_node', anonymous=True)
        self.namespace = rospy.get_namespace()
        rospy.loginfo("namespace: %s", self.namespace)
        # self.ip_robot = rospy.get_param("ip_robot")
        # self.joy_cmd = {
        #     "linear": {
        #         "x": 0.0,
        #         "y": 0.0,
        #         "z": 0.0,
        #     },
        #     "angular": {
        #         "x": 0.0,
        #         "y": 0.0,
        #         "z": 0.0,
        #     }
        # }
        # self.linear_x = 0.0
        # self.linear_y = 0.0
        # self.angular_z = 0.0
        
        self.server_robot_curr_pose_ = rospy.Service(self.namespace +"rpwc_robot_curr_pose", robotMobileBaseState, self.callback_robot_curr_pose)
        self.sub_rpwc_control_poses = rospy.Subscriber(self.namespace + 'rpwc_pose_des', RobotMobileBaseControl, self.callback_rpwc_control_poses)
        self.sub_rpwc_joy_L = rospy.Subscriber(self.namespace + 'joy_web', Twist, self.callback_rpwc_joy)
        # self.sub_rpwc_joy_L = rospy.Subscriber('/setu1/rpwc_joy_L', Twist, self.callback_rpwc_joy_L)
        # self.sub_rpwc_joy_R = rospy.Subscriber('/setu1/rpwc_joy_R', Twist, self.callback_rpwc_joy_R)
        self.pub_curr_pos_ = rospy.Publisher(self.namespace +"rpwc_robot_curr_pose", PoseStamped, queue_size=1)
        self.pose_des = RobotMobileBaseControl()
        self.pose_curr = PoseStamped()


        #rosbridge
        self.client = roslibpy.Ros(host='192.168.131.88', port=9090)
        self.client.run()
        self.pub_poses_control = roslibpy.Topic(self.client, self.namespace + 'poses_control', 'rpwc_msgs/RobotMobileBaseControl')
        self.pub_joy_cmd_vel = roslibpy.Topic(self.client, '/robot/pad_teleop/cmd_vel', 'geometry_msgs/Twist')
        self.sub_odom = roslibpy.Topic(self.client, '/robot/robotnik_base_control/odom', 'nav_msgs/Odometry')
        self.sub_odom.subscribe(self.odom_callback)

        
        
    def callback_robot_curr_pose(self, req):
        robot_curr_pose = self.pose_curr
        robot_curr_pose.header.stamp = rospy.Time.now()
        return robotMobileBaseStateResponse(robot_curr_pose)
  
    def callback_rpwc_control_poses(self, data):
        self.pose_des = data
        message_pose_des = {
            "goalPose": {
                "x": self.pose_des.goalPose.x,
                "y": self.pose_des.goalPose.y,
                "theta": self.pose_des.goalPose.theta,
            },
            "poseDes": {
                "x": self.pose_des.poseDes.x,
                "y": self.pose_des.poseDes.y,
                "theta": self.pose_des.poseDes.theta,
            },
            "poseCurrOff": {
                "x": self.pose_des.poseCurrOff.x,
                "y": self.pose_des.poseCurrOff.y,
                "theta": self.pose_des.poseCurrOff.theta,
            },
            "newSubTask": self.pose_des.newSubTask,
            "morePrecise": self.pose_des.morePrecise
        }
        

        self.pub_poses_control.publish(message_pose_des)
        #rospy.loginfo("Received: %s", self.pose_des)
    
    
    def callback_rpwc_joy(self, data):

        if data.linear.y > 0.3 or data.linear.y < -0.3:
            if data.linear.y > 0.3:
                data.linear.y = data.linear.y - 0.3
            else:
                data.linear.y = data.linear.y + 0.3
        else: 
            data.linear.y = 0

        if data.linear.x > 0.3 or data.linear.x < -0.3:
            if data.linear.x > 0.3:
                data.linear.x = data.linear.x - 0.3
            else:
                data.linear.x = data.linear.x + 0.3
        else: 
            data.linear.x = 0

        if data.angular.z > 0.3 or data.angular.z < -0.3:
            if data.angular.z > 0.3:
                data.angular.z = data.angular.z - 0.3
            else:
                data.angular.z = data.angular.z + 0.3
        else: 
            data.angular.z = 0


        joy_cmd = {
                "linear": {
                    "x": data.linear.y,
                    "y": - data.linear.x,
                    "z": 0.0,
                },
                "angular": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": - data.angular.z,
                }
            }
        self.pub_joy_cmd_vel.publish(joy_cmd)

    # def callback_rpwc_joy_L(self, data):
    #     self.linear_x = data.linear.x
    #     self.linear_y = data.linear.y
    #     self.angular_z = 0.0

    # def callback_rpwc_joy_R(self, data):
    #     self.linear_x = 0.0
    #     self.linear_y = 0.0
    #     self.angular_z = data.angular.z
        
        
    
    def odom_callback(self, message):
        self.pose_curr.header.stamp = rospy.Time.now()
        self.pose_curr.header.frame_id = message['header']['frame_id']
        self.pose_curr.pose.position.x = message['pose']['pose']['position']['x']
        self.pose_curr.pose.position.y = message['pose']['pose']['position']['y']
        self.pose_curr.pose.position.z = message['pose']['pose']['position']['z']
        self.pose_curr.pose.orientation.x = message['pose']['pose']['orientation']['x']
        self.pose_curr.pose.orientation.y = message['pose']['pose']['orientation']['y']
        self.pose_curr.pose.orientation.z = message['pose']['pose']['orientation']['z']
        self.pose_curr.pose.orientation.w = message['pose']['pose']['orientation']['w']
        # self.pub_curr_pos_.publish(self.pose_curr)
        

    def run_class(self):
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():
            # self.pub_joy_cmd_vel.publish(self.joy_cmd)
            self.pub_curr_pos_.publish(self.pose_curr)
            rate.sleep()

        self.pub_poses_control.unadvertise()
        self.client.terminate()

if __name__ == '__main__':
    try:
        custom_node = CustomNode()
        custom_node.run_class()
        

    except rospy.ROSInterruptException:
        pass