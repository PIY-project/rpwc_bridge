#!/usr/bin/env python3

import rospy
from rpwc_msgs.msg import RobotMobileBaseControl
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from rpwc_msgs.srv import robotMobileBaseState, robotMobileBaseStateResponse, stateRec, stateRecResponse, stateExec, stateExecResponse, taskPoc, taskPocRequest, taskPocResponse
from geometry_msgs.msg import PoseStamped


import roslibpy

mobileBaseTask = "Task_0_0"

class CustomNode:
    def __init__(self):
        rospy.init_node('custom_node', anonymous=True)
        self.namespace = rospy.get_namespace()
        rospy.loginfo("namespace: %s", self.namespace)


        self.sub_rpwc_joy = rospy.Subscriber(self.namespace + 'joy_web', Twist, self.callback_rpwc_joy)
        self.serverTeachFromWeb = rospy.Service('/bimuTeachFromWeb', stateRec, self.teachFromWeb)
        self.serverPlayFromWeb = rospy.Service('/bimuPlayFromWeb', stateExec, self.playFromWeb)

        #BIMU ##################
        self.serverPlayFromWeb = rospy.Service('/bimuPlayFromWeb1', stateExec, self.playFromWeb1)
        self.serverPlayFromWeb = rospy.Service('/bimuPlayFromWeb2', stateExec, self.playFromWeb2)
        self.serverPlayFromWeb = rospy.Service('/bimuPlayFromWeb3', stateExec, self.playFromWeb3)
        self.serverPlayFromWeb = rospy.Service('/bimuPlayFromWeb4', stateExec, self.playFromWeb4)
        ############################################


        self.clientStartArmTask = rospy.ServiceProxy('/setup1/set_arm_start_task', taskPoc)
        
        


        self.serverPlayMobileBaseTask = rospy.Service('/setup1/rpwc_mobile_base_task', taskPoc, self.playFromArm)

        #rosbridge
        self.client = roslibpy.Ros(host='192.168.131.88', port=9090)
        self.client.run()
        self.pub_joy_cmd_vel = roslibpy.Topic(self.client, '/robot/pad_teleop/cmd_vel', 'geometry_msgs/Twist')
        self.pub_joy_cmd_vel.advertise()
        
        self.teachCmd = roslibpy.Service(self.client, '/setup1/state_rec', 'rpwc_msgs/stateRec')
        self.playCmd = roslibpy.Service(self.client, '/setup1/state_exec', 'rpwc_msgs/stateExec')

        self.startTaskFromBase = roslibpy.Service(self.client, '/setup1/rpwc_arm_task', 'rpwc_msgs/taskPoc')
        self.startTaskFromBase.advertise(self.handleStartTaskFromBase)
        

        self.response = Bool()
        self.response.data = True


    def handleStartTaskFromBase(self, request, response1):
        print(f'Richiesta ricevuta to start Arm task: {request}')
        name = request['task']['data']
        print(f'Name: {name}')
        number = int(name)
        print(f'Number: {number}')
        # try:
        #     # # Crea la richiesta
        #     requestArm = taskPocRequest()
            
        #     requestArm.sensorID.data = name
        #     # Invia la richiesta e riceve la risposta
        #     responseArm = self.clientStartArmTask(requestArm)
        #     print('fattooo11111111111')
        #     #response1['sensorID']['data'] = "na"
        #     # response['type']['data'] = "0"
        #     # response['subType']['data'] = "0"
        #     print('fattoooooooooooo')
        # except rospy.ServiceException as e:
        #     rospy.logerr("Service call failed: %s", e)

        # print('nooooooooooooo')
        
        # # Crea la richiesta
        requestArm = taskPocRequest()
        
        requestArm.task.data = name
        # Invia la richiesta e riceve la risposta
        self.clientStartArmTask(requestArm)
        print('fattooo11111111111')

        return True


    def playFromArm(self, req):
        request = roslibpy.ServiceRequest({
            'state' : { 'data': 0},
            'TasksExec':[
                {
                    'mode': {'data': 1},
                    'sequenceExecutionType': {'data': 1},
                    'TasksID': [{'data': req.task.data}],
                    'taskInLoop': {'data': False}
                }
            ]
        })
        self.playCmd.call(request)

        print(f'Richiesta ricevuta to start Mobile task: {request}')
        return taskPocResponse() 
        
        
        
    def teachFromWeb(self, req):
        
        request = roslibpy.ServiceRequest({
            'mode' : { 'data': 1},
            'state':{ 'data': req.state.data}
        })

        # # Chiama il servizio e attendi la risposta
        response = self.teachCmd.call(request)
        #rospy.loginfo(f"Richiesta ricevuta:{req.state.data}")
        return stateRecResponse(self.response)
    
    def playFromWeb(self, req):
        
        request = roslibpy.ServiceRequest({
            'state' : { 'data': 0},
            'TasksExec':[
                {
                    'mode': {'data': 1},
                    'sequenceExecutionType': {'data': 1},
                    'TasksID': [{'data': mobileBaseTask}],
                    'taskInLoop': {'data': False}
                }
            ]
        })

        # # Chiama il servizio e attendi la risposta
        response = self.playCmd.call(request)
        #rospy.loginfo(f"Richiesta ricevuta:{req.state.data}")
        return stateExecResponse(self.response)
    
    #BIMU ##################
    def playFromWeb1(self, req):
        
        request = roslibpy.ServiceRequest({
            'state' : { 'data': 0},
            'TasksExec':[
                {
                    'mode': {'data': 1},
                    'sequenceExecutionType': {'data': 1},
                    'TasksID': [{'data': mobileBaseTask}],
                    'taskInLoop': {'data': False}
                }
            ]
        })

        # # Chiama il servizio e attendi la risposta
        response = self.playCmd.call(request)
        #rospy.loginfo(f"Richiesta ricevuta:{req.state.data}")
        return stateExecResponse(self.response)
    
    def playFromWeb2(self, req):
        
        request = roslibpy.ServiceRequest({
            'state' : { 'data': 0},
            'TasksExec':[
                {
                    'mode': {'data': 1},
                    'sequenceExecutionType': {'data': 2},
                    'TasksID': [{'data': mobileBaseTask}],
                    'taskInLoop': {'data': False}
                }
            ]
        })

        # # Chiama il servizio e attendi la risposta
        response = self.playCmd.call(request)
        #rospy.loginfo(f"Richiesta ricevuta:{req.state.data}")
        return stateExecResponse(self.response)
    
    def playFromWeb3(self, req):
        
        request = roslibpy.ServiceRequest({
            'state' : { 'data': 0},
            'TasksExec':[
                {
                    'mode': {'data': 1},
                    'sequenceExecutionType': {'data': 3},
                    'TasksID': [{'data': mobileBaseTask}],
                    'taskInLoop': {'data': False}
                }
            ]
        })

        # # Chiama il servizio e attendi la risposta
        response = self.playCmd.call(request)
        #rospy.loginfo(f"Richiesta ricevuta:{req.state.data}")
        return stateExecResponse(self.response)
    
    def playFromWeb4(self, req):
        
        request = roslibpy.ServiceRequest({
            'state' : { 'data': 0},
            'TasksExec':[
                {
                    'mode': {'data': 1},
                    'sequenceExecutionType': {'data': 4},
                    'TasksID': [{'data': mobileBaseTask}],
                    'taskInLoop': {'data': False}
                }
            ]
        })

        # # Chiama il servizio e attendi la risposta
        response = self.playCmd.call(request)
        #rospy.loginfo(f"Richiesta ricevuta:{req.state.data}")
        return stateExecResponse(self.response)
    
    ################################

    
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