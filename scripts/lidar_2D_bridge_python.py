#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from rpwc_msgs.srv import LaserScanReq, LaserScanReqResponse

import roslibpy



class CustomNodeLidar2D:
    def __init__(self):
        rospy.init_node('custom_node', anonymous=True)
        self.namespace = rospy.get_namespace()
        rospy.loginfo("namespace: %s", self.namespace)
        # self.ip_robot = rospy.get_param("ip_robot")
        
        self.serverLidar2D = rospy.Service(self.namespace +"sensorMeas", LaserScanReq, self.callbackServerLidar2D)
        self.scan = LaserScan()


        #rosbridge
        self.client = roslibpy.Ros(host='192.168.131.88', port=9090)
        self.client.run()
        self.sub_scan = roslibpy.Topic(self.client, '/robot/merged_laser/scan', 'sensor_msgs/LaserScan')
        self.sub_scan.subscribe(self.callback_scan)

        
        
    def callbackServerLidar2D(self, req):
        return LaserScanReqResponse(self.scan)
  
    
    def callback_scan(self, message):
        self.scan.header.stamp = rospy.Time.now()
        self.scan.header.frame_id = message['header']['frame_id']
        self.scan.angle_min = message['angle_min']
        self.scan.angle_max = message['angle_max']
        self.scan.angle_increment = message['angle_increment']
        self.scan.time_increment = message['time_increment']
        self.scan.scan_time = message['scan_time']
        self.scan.range_min = message['range_min']
        self.scan.range_max = message['range_max']
        self.scan.ranges = message['ranges']
        self.scan.intensities = message['intensities']
        

    def run(self):
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown():
            rate.sleep()

        self.client.terminate()

if __name__ == '__main__':
    try:
        custom_node = CustomNodeLidar2D()
        custom_node.run()
        

    except rospy.ROSInterruptException:
        pass