#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import copy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose , PoseWithCovarianceStamped
from itertools import product
from BSA import BSA
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

POSITIONS = [[10.2,17.1]]
# For SOS Lab
#[[10,5],[3.1, 34.6], [15,70.4], [25.1, 30.2],[12.5,30.0]]
# For warehouse
#[[1,1],[10.3,10.4],[15.2,18.1],[10.2,17.1]]

class IBSA(BSA):
    def __init__(self,posX,posY):
        super().__init__(posX,posY)
        self.number_of_sensors = 3 #Number of sensor to inspect
        self.distance_apart = 3 #Assumption of minimal distance beetween two sensor
        self.pixels_apart = 5
    
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
    def expand_gridmap(self):
        coords_to_explore = self.create_radial_offsets_coords(20)
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
                        self.cell_grid.data[(self.x+tmp_x)+((self.y+tmp_y)*self.cell_grid.info.width)] = self.visited
                        
                    self.og_pub.publish(self.cell_grid)
                    self.rate.sleep()
            # If accessing out of grid, just ignore
            except IndexError:
                rospy.loginfo("Error")
                rospy.loginfo(f'x: {tmp_x}, y: {tmp_y} Gridmap info:{self.cartographer_grid.info} index: {self.index}')
                pass
    def flood_fill(self):
        queue = [(0, 0, 0)] # (x, y, distance)
        while queue:
            tmp_x, tmp_y, distance = queue.pop(0)
            
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                new_tmp_x,new_tmp_y = tmp_x+dx,tmp_y+dy
                if self.check_for_obstacle_cell(self.cell_grid,self.cartographer_grid,new_tmp_x,new_tmp_y,self.position_x,self.position_y):
                    self.cell_grid.data[(self.x+new_tmp_x)+((self.y+new_tmp_y)*self.cell_grid.info.width)] = self.obstacle
                if (self.cell_grid.data[(self.x+new_tmp_x)+((self.y+new_tmp_y)*self.cell_grid.info.width)] != self.visited and
                    self.cell_grid.data[(self.x+new_tmp_x)+((self.y+new_tmp_y)*self.cell_grid.info.width)] != self.obstacle and 
                    distance<self.pixels_apart and 
                    self.distance([0,0],[new_tmp_x*self.cell_resolution,new_tmp_y*self.cell_resolution])<self.distance_apart):
                    self.cell_grid.data[(self.x+new_tmp_x)+((self.y+new_tmp_y)*self.cell_grid.info.width)] = self.visited
                    queue.append((new_tmp_x, new_tmp_y, distance + 1))
                if (self.cell_grid.data[(self.x+new_tmp_x)+((self.y+new_tmp_y)*self.cell_grid.info.width)] != self.visited and
                    self.cell_grid.data[(self.x+new_tmp_x)+((self.y+new_tmp_y)*self.cell_grid.info.width)] != self.obstacle and 
                    distance<self.pixels_apart+1 and 
                    self.distance([0,0],[new_tmp_x*self.cell_resolution,new_tmp_y*self.cell_resolution])<self.distance_apart+1):
                    self.cell_grid.data[(self.x+new_tmp_x)+((self.y+new_tmp_y)*self.cell_grid.info.width)] = self.free
            
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()
        return

    def distance(self,p1, p2):
        """
        Calculates the Euclidean distance between two points.
        """
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

        
        

    def check_for_sensor_improved(self):
        res = self.sensorDetection_srv.call()
        if res.success:
            flag = 0
            if len(self.sensor_positions) != 0:
                for i in range(len(self.sensor_positions)):
                    if abs(self.sensor_positions[i][0]-self.position_x)<0.9 and abs(self.sensor_positions[i][1]-self.position_y)<0.9:
                        flag = 1
            if flag == 0:
                self.flood_fill()
                #rospy.loginfo("New sensor detected")
                self.sensor_positions.append([self.position_x,self.position_y])
                #rospy.loginfo(f'Number of sensor detected: {len(self.sensor_positions)}')

    def improvedBSA_loop(self):
        self.state = 0 
        surroundings = self.check_surroundings_2()
        self.update_cellmap(self.x,self.y,surroundings)
        self.state = 1

        while not surroundings[0] and not rospy.is_shutdown():  
            self.state = 1  
            self.check_for_sensor_improved()        
            self.move()
            surroundings = self.check_surroundings_2()
            self.update_cellmap(self.x,self.y,surroundings)
            
        self.state = 2
        #self.move()
        while not rospy.is_shutdown():
            surroundings = self.check_surroundings_2()
            self.update_cellmap(self.x,self.y,surroundings)
            #rospy.loginfo(f'State: {self.state}')
            #rospy.loginfo(f'Obstacles: {surroundings}')
            
            if surroundings == [1,1,0,1]:
                self.check_for_sensor_improved() 
                #rospy.loginfo("Spiral end detected")
                #rospy.loginfo("Attempting to backtrack")
                backtrack = self.backtracking(self.x,self.y,self.cell_grid)
                self.check_for_sensor_improved()
                self.state = 0
                surroundings = self.check_surroundings_2()
                self.update_cellmap(self.x,self.y,surroundings)
                self.state = 1
                surroundings = self.check_surroundings_2()
                if not backtrack:
                    rospy.loginfo("Finished exploration")
                    break
                
            if not surroundings[3]:
                self.turn_left() 
                self.move()
                self.check_for_sensor_improved()
                surroundings = self.check_surroundings_2()
                self.update_cellmap(self.x,self.y,surroundings)

            elif surroundings[0]:
                self.turn_right()
            else:
                self.move()
                self.check_for_sensor_improved()

    
    def improvedmain(self):   
        self.initialize_cell_grid()
        for i in range(100):
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()   
        #self.flood_fill()  
        self.improvedBSA_loop()  
        rospy.loginfo("Finished")
        rospy.loginfo(f"Seconds used: {(rospy.get_time() - self.seconds)}")  
        rospy.loginfo(f"Processing time per node: {(rospy.get_time() - self.seconds)/self.number_of_nodes}")
        rospy.loginfo(f'Sensors detected:{len(self.sensor_positions)}')
        
        for i in range(len(self.path.poses)-1):
            self.distance_traveled += math.sqrt((self.path.poses[i+1].pose.position.x -self.path.poses[i].pose.position.x)**2 + (self.path.poses[i+1].pose.position.y -self.path.poses[i].pose.position.y)**2)

        rospy.loginfo(f'Distance traveled: {self.distance_traveled} m') 

if __name__ == '__main__':
    rospy.init_node("BSA")
    for i in range(len(POSITIONS)):
        bsa = IBSA(POSITIONS[i][0],POSITIONS[i][1])
        bsa.improvedmain()
