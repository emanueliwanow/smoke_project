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
        self.state = 0 # 0:START 1:FORWARD 2:RIGHT 3:BACKWARDS 4:LEFT 5:END

        self.mav = MAV("1")
        self.takeoff_alt = 1        
        self.altitude = 1 
        self.position_x = copy.deepcopy(self.mav.drone_pose.pose.position.x)
        self.position_y = copy.deepcopy(self.mav.drone_pose.pose.position.y)

        self.cell_origin = Pose()
        self.cell_resolution = 0.8
        self.cell_origin.position.x = -(((self.map_size/2)*self.cell_resolution)+(self.cell_resolution/2))+self.position_x
        self.cell_origin.position.y = -(((self.map_size/2)*self.cell_resolution)+(self.cell_resolution/2))+self.position_y
        self.cell_grid = OccupancyGrid()

        self.cartographer_grid = OccupancyGrid()

        self.debug_grid = OccupancyGrid()

        self.obstacle = 100
        self.visited = 50
        self.free = 0
        self.unknown = -1

        self.x = 0
        self.y = 0

        self.obstacle_th = 10

        


        self.og_pub = rospy.Publisher('/scanned_area', OccupancyGrid, queue_size = 5)

        self.debug_grid_pub = rospy.Publisher('/debug_grid', OccupancyGrid, queue_size = 5)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.ScanCallback)
        self.grid_sub = rospy.Subscriber('/map', OccupancyGrid, self.GridCallback)
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
    
    def GridCallback(self, grid_data):
        self.cartographer_grid = grid_data

    def get_costmap_x_y(self,grid, world_x, world_y):
        costmap_x = int(
            round((world_x - grid.info.origin.position.x) / grid.info.resolution))
        costmap_y = int(
            round((world_y - grid.info.origin.position.y) / grid.info.resolution))
        return costmap_x, costmap_y

    def get_costmap_index(self,grid,world_x,world_y):
        costmap_x,costmap_y = self.get_costmap_x_y(grid,world_x,world_y)
        index = costmap_x+(costmap_y*grid.info.width)
        return index

    
    def get_world_x_y(self,grid, costmap_x, costmap_y):
        world_x = costmap_x * grid.info.resolution + grid.info.origin.position.x + grid.info.resolution/2
        world_y = costmap_y * grid.info.resolution + grid.info.origin.position.y + grid.info.resolution/2
        return world_x, world_y

       
    def update_cellmap(self,world_x,world_y,surroundings):
        x,y = self.get_costmap_x_y(self.cell_grid,world_x,world_y)
        self.cell_grid.data[x+((y)*self.cell_grid.info.width)] = self.visited
        rospy.loginfo(f'Center in x:{x} , y:{y}')
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

    def update_cellmap_2(self,x,y,surroundings):
        self.cell_grid.data[x+((y)*self.cell_grid.info.width)] = self.visited
        rospy.loginfo(f'Center in x:{x} , y:{y}')
        if self.state == 0 or not self.state == 3: 
            if surroundings[0]:
                self.cell_grid.data[x+1+(y*self.cell_grid.info.width)] = self.obstacle
            else:
                self.cell_grid.data[x+1+(y*self.cell_grid.info.width)] = self.free
        if self.state == 0 or not self.state == 4:
            if surroundings[1]:
                self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.obstacle
            else:
                self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.free
        if self.state == 0 or not self.state == 1:
            if surroundings[2]:
                self.cell_grid.data[x-1+(y*self.cell_grid.info.width)] = self.obstacle
            else:
                self.cell_grid.data[x-1+(y*self.cell_grid.info.width)] = self.free
        if self.state == 0 or not self.state == 2:
            if surroundings[3]:
                self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.obstacle
            else:
                self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.free

    def update_cellmap_3(self,x,y,surroundings):
        self.cell_grid.data[x+((y)*self.cell_grid.info.width)] = self.visited
        rospy.loginfo(f'Center in x:{x} , y:{y}')
        if self.state == 1:
            if not self.cell_grid.data[x+1+(y*self.cell_grid.info.width)] == self.visited:
                if surroundings[0]:
                    self.cell_grid.data[x+1+(y*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+1+(y*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] == self.visited:
                if surroundings[1]:
                    self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)]  == self.visited:
                if surroundings[3]:
                    self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.free
        if self.state == 2:
            if not self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] == self.visited:
                if surroundings[0]:
                    self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] == self.visited:
                if surroundings[1]:
                    self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)]  == self.visited:
                if surroundings[3]:
                    self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] = self.free
        if self.state == 3:
            if not self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] == self.visited:
                if surroundings[0]:
                    self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] == self.visited:
                if surroundings[1]:
                    self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)]  == self.visited:
                if surroundings[3]:
                    self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] = self.free
        if self.state == 4:
            if not self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] == self.visited:
                if surroundings[0]:
                    self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] == self.visited:
                if surroundings[1]:
                    self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)]  == self.visited:
                if surroundings[3]:
                    self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] = self.free
        
        self.og_pub.publish(self.cell_grid)
        self.rate.sleep()
        

    def check_surroundings(self):
        number_of_scans = len(self.scan.ranges)
        increment = number_of_scans//4
        back = (self.scan.ranges[0] < (3/2)*self.cell_resolution)
        front = (self.scan.ranges[increment*2] < (3/2)*self.cell_resolution)
        left = (self.scan.ranges[increment*3] < (3/2)*self.cell_resolution)
        right = (self.scan.ranges[increment] < (3/2)*self.cell_resolution)
        return [front,right,back,left]

    def check_surroundings_2(self):
        
        index = self.get_costmap_index(self.cartographer_grid,self.position_x,self.position_y)
        num_cell = int(self.cell_grid.info.resolution/round(self.cartographer_grid.info.resolution,5))
        shift = num_cell//2         
        front,right,back,left = 0,0,0,0
        #self.debug_grid = copy.deepcopy(self.cartographer_grid)
        #self.debug_grid.data = list(self.debug_grid.data)
        

        for i in range(num_cell):
            for j in range(num_cell):
                if ((index)+(shift*self.cartographer_grid.info.width-shift)+(i+(j*self.cartographer_grid.info.width))) < len(self.cartographer_grid.data): 
                    if self.cartographer_grid.data[(index)+(shift*self.cartographer_grid.info.width-shift)+(i+(j*self.cartographer_grid.info.width))] > self.obstacle_th:
                        left = 1    
                if ((index)+(-shift*self.cartographer_grid.info.width+shift)+(i+(j*self.cartographer_grid.info.width))) < len(self.cartographer_grid.data):  
                    if self.cartographer_grid.data[(index)+(-shift*self.cartographer_grid.info.width+shift)+(i+(j*self.cartographer_grid.info.width))] > self.obstacle_th:
                        front = 1
                if ((index)+(-3*shift*self.cartographer_grid.info.width-shift)+(i+(j*self.cartographer_grid.info.width))) < len(self.cartographer_grid.data):   
                    if self.cartographer_grid.data[(index)+(-3*shift*self.cartographer_grid.info.width-shift)+(i+(j*self.cartographer_grid.info.width))] > self.obstacle_th:
                        right = 1 
                if ((index)+(-shift*self.cartographer_grid.info.width-3*shift)+(i+(j*self.cartographer_grid.info.width))) < len(self.cartographer_grid.data): 
                    if self.cartographer_grid.data[(index)+(-shift*self.cartographer_grid.info.width-3*shift)+(i+(j*self.cartographer_grid.info.width))] > self.obstacle_th:
                        back = 1 
                 
        #self.debug_grid_pub.publish(self.debug_grid)  
        #self.rate.sleep() 
        if self.state == 0:
            return [front,right,back,left] 
        if self.state == 1:
            return [front,right,back,left]
        if self.state == 2:
            return [right,back,left,front]
        if self.state == 3:
            return [back,left,front,right]
        if self.state == 4:
            return [left,front,right,back]

    def go_front(self):
        self.x += 1
        self.position_x += self.cell_grid.info.resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    
    def go_right(self):
        self.y -= 1
        self.position_y -= self.cell_grid.info.resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    def go_left(self):
        self.y += 1
        self.position_y += self.cell_grid.info.resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    def go_back(self):
        self.x -= 1
        self.position_x -= self.cell_grid.info.resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    def move(self):
        if self.state == 1:
            self.go_front()
        if self.state == 2:
            self.go_right()
        if self.state == 3:
            self.go_back()
        if self.state == 4:
            self.go_left()

    def turn_left(self):
        rospy.loginfo("Turning left")
        if self.state == 1:
            new_state = 4
        if self.state == 2:
            new_state = 1
        if self.state == 3:
            new_state = 2
        if self.state == 4:
            new_state = 3
        self.state = new_state
    
    def turn_right(self):
        rospy.loginfo("Turning right")
        if self.state == 1:
            new_state = 2
        if self.state == 2:
            new_state = 3
        if self.state == 3:
            new_state = 4
        if self.state == 4:
            new_state = 1
        self.state = new_state

    def BSA_loop(self):
        self.state = 1 
        surroundings = self.check_surroundings_2()
        self.x,self.y = self.get_costmap_x_y(self.cell_grid,self.position_x,self.position_y)
        self.update_cellmap_3(self.x,self.y,surroundings)
        self.og_pub.publish(self.cell_grid)

        while all(i==0 for i in surroundings) and not rospy.is_shutdown():  
            self.state = 1          
            self.move()
            self.mav.hold(1)
            surroundings = self.check_surroundings_2()
            self.update_cellmap_3(self.x,self.y,surroundings)
            
        self.state = 2
        self.move()
        while not rospy.is_shutdown():
            surroundings = self.check_surroundings_2()
            self.update_cellmap_3(self.x,self.y,surroundings)
            if all(i==1 for i in surroundings):
                break
            if not surroundings[3]:
                self.turn_left()
                self.move()
                surroundings = self.check_surroundings_2()
                self.update_cellmap_3(self.x,self.y,surroundings)
            if surroundings[0]:
                self.turn_right()
            else:
                self.move()

            
                




        


    


    def main(self):   
        self.initialize_cell_grid()
        for i in range(100):
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()
        self.mav.takeoff_and_keep_yaw(self.takeoff_alt)
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
        #self.mav.hold(1)
        rospy.loginfo("Takeoff finished")     
        self.BSA_loop()  
        self.mav.land()
        self.mav._disarm()          
        
            

if __name__ == '__main__':
    rospy.init_node("BSA")
    bsa = BSA()
    bsa.main()

