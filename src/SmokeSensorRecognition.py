#!/usr/bin/env python3
import rospy
import numpy as np
import math
import time
import copy
from geometry_msgs.msg import PoseStamped, Pose , PoseArray
from std_msgs.msg import Bool



class SmokeSensorRecognition:
    def __init__(self):
        self.sensor_positions = [[0,0]]
        self.rate = rospy.Rate(60)
        self.sensor_positions_pub = rospy.Publisher('/smoke_sensors_postions', PoseArray, queue_size = 5)
        self.sensor_size = 1
        self.drone_pose = PoseStamped()
        self.drone_position_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, self.local_callback)
        rospy.wait_for_message('/mavros/local_position/pose',PoseStamped,timeout=5)
        self.bool_pub = rospy.Publisher('/smoke_sensors_detected', Bool, queue_size = 5)
        self.bool = Bool()
        

    def local_callback(self, local):
        self.drone_pose.pose.position.x = local.pose.position.x
        self.drone_pose.pose.position.y = local.pose.position.y
        self.drone_pose.pose.position.z = local.pose.position.z
        self.drone_pose.pose.orientation.x = local.pose.orientation.x
        self.drone_pose.pose.orientation.y = local.pose.orientation.y
        self.drone_pose.pose.orientation.z = local.pose.orientation.z
        self.drone_pose.pose.orientation.w = local.pose.orientation.w
    def publishing_poses(self):
        pose = Pose()
        poseArray = PoseArray()
        poseArray.header.frame_id = 'map'
        
        for i in range(len(self.sensor_positions)):
            pose.position.x = self.sensor_positions[i][0]
            pose.position.y = self.sensor_positions[i][1]
            poseArray.poses.append(copy.deepcopy(pose))
            
        #print(poseArray)
        for i in range(50):
            self.sensor_positions_pub.publish(poseArray)
            self.rate.sleep()
    def main(self):
        self.publishing_poses()
        while not rospy.is_shutdown():
            for i in range(len(self.sensor_positions)):
                if ((self.drone_pose.pose.position.x < self.sensor_positions[i][0]+self.sensor_size/2 and
                    self.drone_pose.pose.position.x > self.sensor_positions[i][0]-self.sensor_size/2) and
                   (self.drone_pose.pose.position.y < self.sensor_positions[i][1]+self.sensor_size/2 and
                    self.drone_pose.pose.position.y > self.sensor_positions[i][1]-self.sensor_size/2)):
                    self.bool.data = True
                    self.bool_pub.publish(self.bool)
                    #rospy.loginfo("Sensor detected")
                else:
                    self.bool.data = False
                    self.bool_pub.publish(self.bool)
                self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("SmokeSensorRecognition")
    smoke = SmokeSensorRecognition()
    smoke.main()
