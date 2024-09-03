#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import copy
from mavbase.MAV import MAV
from sensor_msgs.msg import LaserScan
from SweepSearcher import planning
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class BoustrophedonPlanner:
    def __init__(self):
        self.scan = LaserScan()
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.ScanCallback)

        self.path_pub = rospy.Publisher('/2d_path', Path, queue_size = 5)
        
        self.path = Path()
        self.pose = PoseStamped()
        self.path.header.frame_id = 'map'
    
    def ScanCallback(self, scan_data):
        self.scan = scan_data
    
    def PathGen(self, init_x = 0, init_y = 0):
        number_of_scans = len(self.scan.ranges)
        increment = number_of_scans//4
        x1 = -self.scan.ranges[0] + init_x
        y1 = -self.scan.ranges[increment] + init_y
        x2 = self.scan.ranges[increment*2] + init_x
        y2 = -self.scan.ranges[increment] + init_y
        x3 = self.scan.ranges[increment*2] + init_x
        y3 = self.scan.ranges[increment*3] + init_y
        x4 = -self.scan.ranges[0] + init_x
        y4 = self.scan.ranges[increment*3] + init_y

        ox = [x1, x2, x3, x4, x1]
        oy = [y1, y2, y3, y4, y1]
        rospy.loginfo(f"Square Polygon in x {ox} , y  {oy}")

        resolution = 0.8

        px, py = planning(ox, oy, resolution)
        pose_list = []
        for i in range(len(px)):
            self.pose.pose.position.x = px[i]
            self.pose.pose.position.y = py[i]
            self.pose.pose.position.z = 0
            pose_list.append(copy.deepcopy(self.pose))

        self.path.poses = pose_list
        self.path_pub.publish(self.path)

        return px,py
        
        
    
    def go(self):

        rospy.init_node("mav_test")
        mav = MAV("1")

        px, py = self.PathGen(mav.drone_pose.pose.position.x,mav.drone_pose.pose.position.y)

        takeoff_alt = 1
        
        altitude = 1

        
        mav.takeoff_and_keep_yaw(takeoff_alt)
        mav.hold(4)
        rospy.loginfo("Takeoff finished")

        for i in range(len(px)):
            rospy.loginfo(f"Going to x: {px[i]} y: {py[i]}")

            mav.set_position_with_yaw(px[i],py[i],altitude) 
            
        
        
        #mav.set_altitude(1)
        mav.land()
        mav._disarm()

if __name__ == "__main__":
    test = BoustrophedonPlanner()
    test.go()