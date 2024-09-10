#!/usr/bin/env python

import rospy
from rpwc_msgs.msg import RobotMobileBaseControl
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyRequest
from rpwc_msgs.srv import robotMobileBaseState, robotMobileBaseStateResponse, setHardwareActivation, setHardwareActivationRequest, setHardwareActivationResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import roslibpy

class CustomNode:
    def __init__(self):
        rospy.init_node('custom_node', anonymous=True)
        self.namespace = rospy.get_namespace()
        setup_namespace = rospy.get_param('setup_namespace','/setup1')
        rospy.loginfo("namespace: %s", self.namespace)
        rospy.loginfo("setup_namespace: %s", setup_namespace)
        
        rospy.wait_for_service(setup_namespace + '/set_hardware_activation')
        set_hardware_activation_service = rospy.ServiceProxy(setup_namespace + '/set_hardware_activation', setHardwareActivation)
        
        #Servizio per lanciare in play il braccio
        self.client = roslibpy.Ros(host='192.168.131.22', port=9090)
        self.client.run()
        self.start_arm_task = roslibpy.Service(self.client, setup_namespace + '/set_arm_start_task', 'std_msgs/Empty')

        #Servizio che legge su rpwc onboard della base mobile e manda un rolibpy per play braccio
        self.server_launch_arm_task_ = rospy.Service(setup_namespace +"/rpwc_arm_task", Empty, self.callback_launch_arm_task)

        # Prepara il messaggio di richiesta
        req = setHardwareActivationRequest()
        req.hardwareID.data = 'myBaseMobile'
        req.activation.data = True
        # Chiamata al servizio
        res = set_hardware_activation_service(req)

            # Stampa il risultato
        if res.result.data:
            rospy.loginfo("Hardware activation success")
        else:
            rospy.logwarn("Hardware activation failed")


        self.server_robot_curr_pose_ = rospy.Service(self.namespace +"rpwc_robot_curr_pose", robotMobileBaseState, self.callback_robot_curr_pose)
     
        self.sub_rpwc_control_poses = rospy.Subscriber(self.namespace + 'rpwc_pose_des', RobotMobileBaseControl, self.callback_rpwc_control_poses)


        self.pub_curr_pos_ = rospy.Publisher(self.namespace +"rpwc_robot_curr_pose", PoseStamped, queue_size=1)
        self.pose_des = RobotMobileBaseControl()
        self.pose_curr = PoseStamped()


       
        self.pub_poses_control = rospy.Publisher(self.namespace + 'poses_control', RobotMobileBaseControl, queue_size=1)
        self.sub_odom = rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.odom_callback)

    def callback_launch_arm_task(self,req):
        #invia empy msg
        # Prepara il messaggio di richiesta
        #ANORMALE CON ROSLIBPY
        request = roslibpy.ServiceRequest({
            
        })
         # # Chiama il servizio e attendi la risposta
        response = self.start_arm_task.call(request)
        return EmptyRequest(self.response)



    def callback_robot_curr_pose(self, req):
        robot_curr_pose = self.pose_curr
        robot_curr_pose.header.stamp = rospy.Time.now()
        return robotMobileBaseStateResponse(robot_curr_pose)
  
    def callback_rpwc_control_poses(self, data):
        self.pose_des = data   
        self.pub_poses_control.publish(self.pose_des)
        #rospy.loginfo("Received: %s", self.pose_des)
    
  
    
    def odom_callback(self, data):
        self.pose_curr.header.stamp = rospy.Time.now()
        self.pose_curr.header.frame_id = data.header.frame_id
        self.pose_curr.pose.position.x = data.pose.pose.position.x
        self.pose_curr.pose.position.y = data.pose.pose.position.y
        self.pose_curr.pose.position.z = data.pose.pose.position.z
        self.pose_curr.pose.orientation.x = data.pose.pose.orientation.x
        self.pose_curr.pose.orientation.y =  data.pose.pose.orientation.y
        self.pose_curr.pose.orientation.z =  data.pose.pose.orientation.z
        self.pose_curr.pose.orientation.w =  data.pose.pose.orientation.w

    def run_class(self):
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():
            # self.pub_joy_cmd_vel.publish(self.joy_cmd)
            self.pub_curr_pos_.publish(self.pose_curr)
            rate.sleep()

        self.pub_poses_control.unadvertise()


   

if __name__ == '__main__':
    try:
        custom_node = CustomNode()        
        custom_node.run_class()

    except rospy.ROSInterruptException:
        pass