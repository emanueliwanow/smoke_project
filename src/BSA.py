#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import copy
from mavbase.MAV import MAV
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose


class BSA:
    def __init__(self):
        self.scan = LaserScan()
        self.occupancy_grid = OccupancyGrid()
        self.map_size = 50
        self.rate = rospy.Rate(60)
        self.scan = LaserScan()
        self.costmapPose_x = 0
        self.costmapPose_y = 0
        self.direction = 0 # 0:FORWARD 1:RIGHT 2:BACKWARDS 3:LEFT 

        self.mav = MAV("1")
        self.takeoff_alt = 1        
        self.altitude = 1 
        self.position_x = self.mav.drone_pose.pose.position.x
        self.position_y = self.mav.drone_pose.pose.position.y

        self.cell_origin = Pose()
        self.cell_resolution = 0.8
        self.cell_origin.position.x = -(((self.map_size/2)*self.cell_resolution)+(self.cell_resolution/2))+self.position_x
        self.cell_origin.position.y = -(((self.map_size/2)*self.cell_resolution)+(self.cell_resolution/2))+self.position_y
        self.cell_grid = OccupancyGrid()

        

        self.obstacle = 100
        self.visited = 50
        self.free = 0
        self.unknown = -1

        

        


        self.og_pub = rospy.Publisher('/scanned_area', OccupancyGrid, queue_size = 5)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.ScanCallback)
        rospy.wait_for_message('/scan', LaserScan, timeout=5)

    
    def initialize_cell_grid(self):
        self.cell_grid.header.frame_id = 'map'
        self.cell_grid.info.resolution = self.cell_resolution
        self.cell_grid.info.width = self.map_size
        self.cell_grid.info.height = self.map_size
        self.cell_grid.info.origin.position.x = self.cell_origin.position.x
        self.cell_grid.info.origin.position.y = self.cell_origin.position.y
        self.cell_grid.data = [self.unknown for i in range(self.map_size**2)]

    def ScanCallback(self, scan_data):
        self.scan = scan_data

    def get_costmap_x_y(self, world_x, world_y):
        costmap_x = int(
            round((world_x - self.cell_origin.position.x) / self.cell_resolution))
        costmap_y = int(
            round((world_y - self.cell_origin.position.y) / self.cell_resolution))
        return costmap_x, costmap_y
    
    def get_world_x_y(self, costmap_x, costmap_y):
        world_x = costmap_x * self.cell_resolution + self.cell_origin.position.x
        world_y = costmap_y * self.cell_resolution + self.cell_origin.position.y
        return world_x, world_y

       
    def update_cellmap(self,world_x,world_y,surroundings):
        x,y = self.get_costmap_x_y(world_x,world_y)
        self.cell_grid.data[x+((y)*self.cell_grid.info.width)] = self.visited
        if surroundings[0]:
            self.cell_grid.data[x+1+(y*self.cell_grid.info.width)] = self.obstacle
        else:
            self.cell_grid.data[x+1+(y*self.cell_grid.info.width)] = self.free
        if surroundings[1]:
            self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.obstacle
        else:
            self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.free
        if surroundings[2]:
            self.cell_grid.data[x-1+(y*self.cell_grid.info.width)] = self.obstacle
        else:
            self.cell_grid.data[x-1+(y*self.cell_grid.info.width)] = self.free
        if surroundings[3]:
            self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.obstacle
        else:
            self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.free

    def check_surroundings(self):
        number_of_scans = len(self.scan.ranges)
        increment = number_of_scans//4
        back = (self.scan.ranges[0] < (3/2)*self.cell_resolution)
        front = (self.scan.ranges[increment*2] < (3/2)*self.cell_resolution)
        left = (self.scan.ranges[increment*3] < (3/2)*self.cell_resolution)
        right = (self.scan.ranges[increment] < (3/2)*self.cell_resolution)
        return [front,right,back,left]

    def go_front(self):
        self.position_x += self.cell_resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    
    def go_right(self):
        self.position_y -= self.cell_resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    def go_left(self):
        self.position_y += self.cell_resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    def go_back(self):
        self.position_x -= self.cell_resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)

    def BSA_loop(self):
        surroundings = self.check_surroundings()
        self.update_cellmap(self.position_x,self.position_y,surroundings)
        self.og_pub.publish(self.cell_grid)

        while all(i==0 for i in surroundings) and not rospy.is_shutdown():            
            self.go_front()
            self.mav.hold(1)
            surroundings = self.check_surroundings()
            self.update_cellmap(self.position_x,self.position_y,surroundings)
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()


        


    


    def main(self):   
        self.initialize_cell_grid()
        for i in range(100):
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()
        self.mav.takeoff_and_keep_yaw(self.takeoff_alt)
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
        self.mav.hold(1)
        rospy.loginfo("Takeoff finished")     
        self.BSA_loop()  
        self.mav.land()
        self.mav._disarm()          
        
            

if __name__ == '__main__':
    rospy.init_node("BSA")
    bsa = BSA()
    bsa.main()

