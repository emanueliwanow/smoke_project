#!/usr/bin/env python3
import rospy
import numpy as np
import math
import time
import copy
from geometry_msgs.msg import PoseStamped, Pose , PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import Trigger


class SmokeSensorRecognition:
    def __init__(self):
        # For the hole lab
        #self.sensor_positions = [[8.5, 3.4], [8.6, 7.4], [8.6, 10.7], [8.6, 14.1], [8.6, 17.5], [8.6, 20.7], [8.7, 24.0], [8.6, 27.4], [8.6, 30.7], [8.5, 34.0], [13.1, 3.3], [15.9, 3.3], [19.0, 3.4], [15.8, 7.1], [15.8, 9.6], [13.6, 14.0], [13.4, 18.4], [17.9, 14.0], [17.8, 18.5], [15.7, 22.5], [15.5, 25.7], [15.5, 28.2], [15.5, 30.5], [15.5, 32.9], [15.4, 36.6], [15.4, 40.6], [15.5, 44.0], [15.4, 47.4], [22.4, 4.0], [25.6, 4.1], [24.0, 9.1], [24.1, 12.3], [23.6, 16.5], [22.7, 20.2], [22.6, 22.7], [22.7, 25.8], [22.7, 29.0], [22.7, 32.0], [29.8, 30.8], [29.8, 34.3], [29.9, 37.4], [29.7, 41.4], [29.5, 45.6], [29.5, 49.7], [29.5, 53.9], [29.4, 58.2], [22.5, 49.0], [22.3, 52.4], [22.3, 55.7], [22.4, 59.2], [22.6, 64.0], [28.1, 63.9], [20.3, 70.0], [25.6, 70.1], [33.1, 63.5], [35.9, 60.9], [39.7, 60.3], [39.5, 63.7], [31.1, 69.4], [34.8, 69.5], [40.3, 69.4], [43.0, 61.3], [46.3, 61.4], [49.6, 61.4], [44.6, 69.5], [48.0, 69.5], [51.2, 69.5], [53.1, 61.4], [53.8, 69.0], [56.4, 59.5], [59.5, 62.9], [58.9, 66.0], [54.4, 66.0], [49.4, 65.8], [43.8, 66.0], [37.6, 65.9], [31.9, 65.9], [25.9, 58.1], [25.9, 52.5], [22.3, 46.1], [26.1, 44.1], [26.1, 36.6], [19.1, 42.7], [19.0, 36.9], [19.0, 29.8], [19.0, 23.3], [12.0, 27.8]]
        # For the SOS lab only
        #self.sensor_positions = [[4.8, 3.3], [4.7, 7.4], [4.8, 10.8], [4.8, 14.1], [4.7, 17.4], [4.7, 20.7], [4.7, 24.1], [4.7, 27.4], [4.7, 30.8], [4.7, 34.1], [9.3, 3.3], [14.0, 3.4], [12.0, 9.6], [12.0, 7.0], [8.2, 9.2], [9.1, 13.7], [9.1, 18.4], [13.9, 13.6], [14.1, 18.4], [11.6, 22.4], [11.6, 26.0], [11.6, 28.3], [11.6, 30.5], [8.0, 25.4], [11.2, 32.9], [11.6, 36.5], [11.6, 40.6], [11.4, 44.0], [11.6, 47.4], [20.0, 5.6], [20.2, 9.0], [20.5, 12.4], [19.7, 16.4], [19.4, 20.2], [18.8, 22.7], [18.9, 25.7], [18.9, 29.0], [15.2, 26.8], [18.7, 31.9], [15.1, 38.6], [18.3, 46.2], [18.4, 49.0], [18.6, 52.3], [18.7, 55.7], [18.4, 59.1], [18.8, 64.0], [25.1, 63.8], [16.0, 70.1], [25.6, 58.1], [25.7, 53.9], [25.7, 49.8], [25.5, 45.5], [25.6, 41.6], [25.9, 37.3], [25.8, 34.3], [25.8, 30.9], [22.2, 38.7]]
        # For warehouse
        self.sensor_positions = [[2.6, 2.3], [2.7, 7.5], [2.7, 11.9], [2.8, 17.1], [6.3, 7.6], [6.2, 11.9], [6.2, 2.4], [6.5, 17.2], [9.8, 2.4], [9.7, 7.5], [9.6, 12.1], [9.8, 17.3], [13.4, 2.4], [13.5, 7.3], [13.4, 12.2], [13.3, 17.4], [17.1, 2.5], [17.4, 7.3], [17.2, 12.3], [17.2, 17.4], [21.7, 2.5], [21.8, 7.4], [21.8, 12.4], [22.0, 17.5]]

        self.rate = rospy.Rate(60)
        self.sensor_positions_pub = rospy.Publisher('/smoke_sensors_postions', PoseArray, queue_size = 5)
        self.sensor_size = 0.45
        self.drone_pose = PoseStamped()
        self.drone_position_sub = rospy.Subscriber('/pose',PoseStamped, self.local_callback)
        #rospy.wait_for_message('/mavros/local_position/pose',PoseStamped,timeout=5)
        
        self.detection_srv = rospy.Service('/sensor_detection', Trigger, self.handleDetectionSrv)

        self.localSmoke_sub = rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,self.localSmoke_callback)
        self.localSmoke = PoseWithCovarianceStamped()

        self.marker_pub = rospy.Publisher('/marker', MarkerArray,queue_size=5)
        self.id = 0
        self.markerArray = MarkerArray()
        
    def handleDetectionSrv(self,req):
        flag = 0
        for i in range(len(self.sensor_positions)):
            if (abs(self.drone_pose.pose.position.x-self.sensor_positions[i][0])<self.sensor_size and
                abs(self.drone_pose.pose.position.y-self.sensor_positions[i][1])<self.sensor_size):
                flag = 1
            
        return flag, ""                
            

    def localSmoke_callback(self,local):
        self.localSmoke = local
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.localSmoke.pose.pose.position.x
        marker.pose.position.y = self.localSmoke.pose.pose.position.y
        marker.pose.position.z = 0
        marker.id = self.id 
        self.markerArray.markers.append(marker)
        self.marker_pub.publish(self.markerArray)
        self.id +=1
        self.sensor_positions.append([round(self.localSmoke.pose.pose.position.x,1),round(self.localSmoke.pose.pose.position.y,1)])
        print(self.sensor_positions)

    def local_callback(self, local):
        self.drone_pose = local

    
    #def place_smokeSensor_loop(self):


    def publishing_poses(self):
        
        
        for i in range(len(self.sensor_positions)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.sensor_positions[i][0]
            marker.pose.position.y = self.sensor_positions[i][1]
            marker.pose.position.z = 0
            marker.id = self.id 
            self.markerArray.markers.append(marker)
            self.marker_pub.publish(self.markerArray)
            self.id +=1
            
    def main(self):
        rospy.sleep(3)
        rospy.loginfo(f'Number of sensors:{len(self.sensor_positions)}')
        self.publishing_poses()
        while not rospy.is_shutdown():            
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("SmokeSensorRecognition")
    smoke = SmokeSensorRecognition()
    smoke.main()
