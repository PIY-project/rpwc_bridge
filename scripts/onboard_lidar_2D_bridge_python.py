#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from rpwc_msgs.srv import LaserScanReq, LaserScanReqResponse, setHardwareActivation, setHardwareActivationRequest, setHardwareActivationResponse



class CustomNodeLidar2D:
    def __init__(self):
        rospy.init_node('custom_node', anonymous=True)
        self.namespace = rospy.get_namespace()
        rospy.loginfo("namespace: %s", self.namespace)
        # self.ip_robot = rospy.get_param("ip_robot")
        setup_namespace = rospy.get_param('setup_namespace','/setup1')
 
        rospy.wait_for_service(setup_namespace + '/set_hardware_activation')
        set_hardware_activation_service = rospy.ServiceProxy(setup_namespace + '/set_hardware_activation', setHardwareActivation)
# Prepara il messaggio di richiesta
        req = setHardwareActivationRequest()
        req.hardwareID.data = 'myLidar'
        req.activation.data = True
 # Chiamata al servizio
        res = set_hardware_activation_service(req)

            # Stampa il risultato
        if res.result.data:
            rospy.loginfo("Hardware activation success")
        else:
            rospy.logwarn("Hardware activation failed")

        self.serverLidar2D = rospy.Service(self.namespace +"sensorMeas", LaserScanReq, self.callbackServerLidar2D)
        self.scan = LaserScan()



        self.sub_scan = rospy.Subscriber('/robot/merged_laser/scan', LaserScan, self.callback_scan)

        
        
    def callbackServerLidar2D(self, req):
        return LaserScanReqResponse(self.scan)
  
    
    def callback_scan(self, data):
        self.scan.header.stamp = rospy.Time.now()
        self.scan.header.frame_id = data.header.frame_id
        self.scan.angle_min = data.angle_min
        self.scan.angle_max = data.angle_max
        self.scan.angle_increment = data.angle_increment
        self.scan.time_increment = data.time_increment
        self.scan.scan_time = data.scan_time
        self.scan.range_min = data.range_min
        self.scan.range_max = data.range_max
        self.scan.ranges = data.ranges
        self.scan.intensities = data.intensities
        

    def run(self):
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        custom_node = CustomNodeLidar2D()
        custom_node.run()
        

    except rospy.ROSInterruptException:
        pass