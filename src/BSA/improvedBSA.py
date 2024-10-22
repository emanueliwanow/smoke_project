#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import copy
from mavbase.MAV import MAV
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose , PoseWithCovarianceStamped
from cartographer_ros_msgs.msg import SubmapTexture
from cartographer_ros_msgs.srv import SubmapQuery
from itertools import product
from BSA import BSA
from std_msgs.msg import Bool

class improvedBSA(BSA):
    def __init__(self):
        super().__init__()
        self.number_of_sensors = 3 #Number of sensor to inspect
        self.distance_apart = 10 #Assumption of minimal distance beetween two sensor
        self.inspected_sensors = 0
        self.smoke_sensor_recognition_sub = rospy.Subscriber('/smoke_sensors_detected',Bool, self.smoke_sensor_callback)

        self.sensor_detected = Bool()

   
    def smoke_sensor_callback(self,data):
        self.sensor_detected.data = data.data
        #rospy.loginfo(f"Sensor: {data.data}")
 
        
        



    def create_radial_offsets_coords(self,radius):
            coords = {}
            # iterate increasing over every radius value...
            for r in range(1, radius + 1):
                # for this radius value... (both product and range are generators too)
                tmp_coords = product(range(-r, r + 1), repeat=2)
                # only yield new coordinates
                for i, j in tmp_coords:
                    if (i, j) != (0, 0) and not coords.get((i, j), False):
                        coords[(i, j)] = True
                        yield (i, j)
    def check_for_obstacle_cell(self,grid,cartographer_grid,x,y,world_x,world_y):
        index = self.get_costmap_index(cartographer_grid,world_x,world_y)
        self.index = index
        num_cell = int(grid.info.resolution/round(cartographer_grid.info.resolution,5))
        shift = num_cell//2  
        index_x = index%cartographer_grid.info.width   
        index_y = index//cartographer_grid.info.width
        num_cell_x = int((grid.info.resolution*x)/round(cartographer_grid.info.resolution,5))
        num_cell_y = int((grid.info.resolution*y)/round(cartographer_grid.info.resolution,5))
        for i in range(num_cell):
            for j in range(num_cell):
                if (index_x+num_cell_x-shift+i)<cartographer_grid.info.width and (index_y+num_cell_y-shift+j)<cartographer_grid.info.height and (index_x+num_cell_x-shift+i)>=0 and (index_y+num_cell_y-shift+j)>=0:
                    if cartographer_grid.data[(index)+(num_cell_x-shift+i)+(num_cell_y-shift+j)*cartographer_grid.info.width] >self.obstacle_th:
                        return True
        return False
    def check_for_obstacle_previous_cell(self,grid,x,y,tmp_x,tmp_y):
        discretization = 100
        obstacle = 0
        x_dis = np.linspace(0, tmp_x, discretization)
        if tmp_x == 0:
            y_dis = np.linspace(0, tmp_y, discretization)
        else:
            y_dis = x_dis*(tmp_y/tmp_x)
        for i in range(discretization):
            if tmp_x>0:
                value_x = x_dis[i]-(grid.info.resolution/2)
                index_x = int(value_x//grid.info.resolution +1)
            if tmp_x == 0:
                index_x = 0
            if tmp_x<0:
                value_x = x_dis[i]+(grid.info.resolution/2)
                index_x = int(value_x//grid.info.resolution - 1)
            if tmp_y>0:
                value_y = y_dis[i]-(grid.info.resolution/2)
                index_y = int(value_y//grid.info.resolution +1)
            if tmp_y == 0:
                index_y = 0
            if tmp_y<0:
                value_y = y_dis[i]+(grid.info.resolution/2)
                index_y = int(value_y//grid.info.resolution -1)
            if grid.data[(x+index_x)+((y+index_y)*grid.info.width)] == self.obstacle:
                obstacle = 1
        return obstacle
            

    def expand_gridmap(self):
        coords_to_explore = self.create_radial_offsets_coords(40)
        self.cell_grid.data[(self.x)+((self.y)*self.cell_grid.info.width)] = self.visited
        self.og_pub.publish(self.cell_grid)
        self.rate.sleep()
        

        for idx, radius_coords in enumerate(coords_to_explore):
            tmp_x, tmp_y = radius_coords            
            try:
                
                if (((abs(tmp_x)+(1/2))*self.cell_resolution)**2)+(((abs(tmp_y)+(1/2))*self.cell_resolution)**2)<=self.distance_apart**2:
                    if self.check_for_obstacle_cell(self.cell_grid,self.cartographer_grid,tmp_x,tmp_y,self.position_x,self.position_y):
                        self.cell_grid.data[(self.x+tmp_x)+((self.y+tmp_y)*self.cell_grid.info.width)] = self.obstacle
                    else:
                        if not self.check_for_obstacle_previous_cell(self.cell_grid,self.x,self.y,tmp_x,tmp_y):
                            self.cell_grid.data[(self.x+tmp_x)+((self.y+tmp_y)*self.cell_grid.info.width)] = self.visited
                        
                            
                                
                                
                    #rospy.loginfo("Update")
                    self.og_pub.publish(self.cell_grid)
                    self.rate.sleep()
            # If accessing out of grid, just ignore
            except IndexError:
                rospy.loginfo("Error")
                rospy.loginfo(f'x: {tmp_x}, y: {tmp_y} Gridmap info:{self.cartographer_grid.info} index: {self.index}')
                pass


    def improvedBSA_loop(self):
        self.state = 0 
        surroundings = self.check_surroundings_2()
        self.update_cellmap_3(self.x,self.y,surroundings)
        self.state = 1

        while not surroundings[0] and not rospy.is_shutdown():  
            if self.sensor_detected.data:
                rospy.loginfo("Sensor detected, expanding gridmap")
                self.expand_gridmap()
            self.state = 1          
            self.move()
            self.mav.hold(1)
            surroundings = self.check_surroundings_2()
            self.update_cellmap_3(self.x,self.y,surroundings)
            
        self.state = 2
        #self.move()
        while not rospy.is_shutdown():
            surroundings = self.check_surroundings_2()
            self.update_cellmap_3(self.x,self.y,surroundings)
            #rospy.loginfo(f'State: {self.state}')
            #rospy.loginfo(f'Obstacles: {surroundings}')
            
            if surroundings == [1,1,0,1]:
                rospy.loginfo("Spiral end detected")
                rospy.loginfo("Attempting to backtrack")
                backtrack = self.backtracking(self.x,self.y,self.cell_grid)
                self.state = 0
                surroundings = self.check_surroundings_2()
                self.update_cellmap_3(self.x,self.y,surroundings)
                self.state = 1
                surroundings = self.check_surroundings_2()
                if not backtrack:
                    rospy.loginfo("Finished exploration")
                    break
                
            if not surroundings[3]:
                self.turn_left()
                self.move()
                surroundings = self.check_surroundings_2()
                self.update_cellmap_3(self.x,self.y,surroundings)

            elif surroundings[0]:
                self.turn_right()
            else:
                self.move()
    
    def improvedmain(self):   
        self.initialize_cell_grid()
        for i in range(100):
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()
        #self.mav.takeoff_and_keep_yaw(self.takeoff_alt)
        #self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
        #self.mav.hold(5)
        #rospy.loginfo("Takeoff finished")     
        #self.improvedBSA_loop()
        self.expand_gridmap()  
        #self.mav.land()
        #self.mav._disarm() 
        #rospy.loginfo(f"Seconds used: {(rospy.get_time() - self.seconds)}") 

if __name__ == '__main__':
    rospy.init_node("BSA")
    bsa = improvedBSA()
    bsa.improvedmain()
