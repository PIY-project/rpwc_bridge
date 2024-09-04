#!/usr/bin/env python

import rospy
from rpwc_msgs.msg import RobotMobileBaseControl
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rpwc_msgs.srv import robotMobileBaseState, robotMobileBaseStateResponse, stateRec, stateRecResponse
from geometry_msgs.msg import PoseStamped


import roslibpy



class CustomNode:
    def __init__(self):
        rospy.init_node('custom_node', anonymous=True)
        self.namespace = rospy.get_namespace()
        rospy.loginfo("namespace: %s", self.namespace)

        self.sub_rpwc_joy = rospy.Subscriber(self.namespace + 'joy_web', Twist, self.callback_rpwc_joy)
        self.serverTeachFromWeb = rospy.Service('/bimuTeachFromWeb', stateRec, self.teachFromWeb)
        

        #rosbridge
        self.client = roslibpy.Ros(host='192.168.131.88', port=9090)
        self.client.run()
        self.pub_joy_cmd_vel = roslibpy.Topic(self.client, '/robot/pad_teleop/cmd_vel', 'geometry_msgs/Twist')
        self.teachCmd = roslibpy.Service(self.client, '/setup1/state_rec', 'rpwc_msgs/stateRec')

        self.response = Bool()
        self.response.data = True

        # # Definisci il client del servizio

       
        
        
        
    def teachFromWeb(self, req):
        
        request = roslibpy.ServiceRequest({
            'mode' : { 'data': 1},
            'state':{ 'data': req.state.data}
        })

        # Chiama il servizio e attendi la risposta
        response = self.teachCmd.call(request)
        #rospy.loginfo(f"Richiesta ricevuta:{req.state.data}")
        return stateRecResponse(self.response)
    
    
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


        
    
    
        

    def run_class(self):
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():

            rate.sleep()

        self.client.terminate()

if __name__ == '__main__':
    try:
        custom_node = CustomNode()
        custom_node.run_class()
        

    except rospy.ROSInterruptException:
        pass