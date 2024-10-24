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
from mavbase.MAV import MAV


class droneBSA(BSA):
    def __init__(self,posX=0,posY=0):
        super().__init__(posX,posY)
        self.mav = MAV("1")
        self.takeoff_alt = 1        
        self.altitude = 1 

    def drone_get_closest_cell_withAstar(self, x, y, grid, desired_cost = 0, max_radius=80):

        def create_radial_offsets_coords(radius):
            """
            Creates an ordered by radius (without repetition)
            generator of coordinates to explore around an initial point 0, 0

            For example, radius 2 looks like:
            [(-1, -1), (-1, 0), (-1, 1), (0, -1),  # from radius 1
            (0, 1), (1, -1), (1, 0), (1, 1),  # from radius 1
            (-2, -2), (-2, -1), (-2, 0), (-2, 1),
            (-2, 2), (-1, -2), (-1, 2), (0, -2),
            (0, 2), (1, -2), (1, 2), (2, -2),
            (2, -1), (2, 0), (2, 1), (2, 2)]
            """
            # We store the previously given coordinates to not repeat them
            # we use a Dict as to take advantage of its hash table to make it more efficient
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

        coords_to_explore = create_radial_offsets_coords(max_radius)
        shortest_path = None
        lenght = 0
        goal_x = 0
        goal_y = 0
        for idx, radius_coords in enumerate(coords_to_explore):
            # for coords in radius_coords:
            tmp_x, tmp_y = radius_coords
            # print("Checking coords: " +
            #       str((x + tmp_x, y + tmp_y)) +
            #       " (" + str(idx) + " / " + str(len(coords_to_explore)) + ")")
            try:
                cost = self.get_cost_from_costmap_x_y(x + tmp_x, y + tmp_y,grid)
            # If accessing out of grid, just ignore
            except IndexError:
                pass
            self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
            if int(cost) == int(desired_cost):
                self.astar_goal.pose.position.x = self.position_x + self.cell_grid.info.resolution*(tmp_x)
                self.astar_goal.pose.position.y = self.position_y + self.cell_grid.info.resolution*(tmp_y)
                self.astar_initial.pose.pose.position.x = self.position_x
                self.astar_initial.pose.pose.position.y = self.position_y
                path,flag = self.AstarGetPath(self.astar_initial,self.astar_goal)
                if flag:
                    temp_lenght = 0 
                    for i in range(len(path.poses)-1):
                        temp_lenght += math.sqrt(pow((path.poses[i+1].pose.position.x - path.poses[i].pose.position.x),2) + pow((path.poses[i+1].pose.position.y - path.poses[i].pose.position.y), 2))
                    if shortest_path == None:
                        shortest_path = path
                        lenght = temp_lenght
                        goal_x = x+tmp_x
                        goal_y = y+tmp_y
                    else:
                        if temp_lenght < lenght:
                            shortest_path = path
                            lenght = temp_lenght
                            goal_x = x+tmp_x
                            goal_y = y+tmp_y
        
        return shortest_path,goal_x,goal_y
    def drone_go_front(self):
        self.x += 1
        self.position_x += self.cell_grid.info.resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    
    def drone_go_right(self):
        self.y -= 1
        self.position_y -= self.cell_grid.info.resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    def drone_go_left(self):
        self.y += 1
        self.position_y += self.cell_grid.info.resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    def drone_go_back(self):
        self.x -= 1
        self.position_x -= self.cell_grid.info.resolution
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
    def drone_move(self):
        if self.state == 1:
            self.drone_go_front()
        if self.state == 2:
            self.drone_go_right()
        if self.state == 3:
            self.drone_go_back()
        if self.state == 4:
            self.drone_go_left()
    def drone_backtracking(self,x,y,grid):
        # Backtracking without obstacle avoidance
        shortest_path,goal_x,goal_y = self.get_closest_cell_withAstar(x,y,grid)
        if shortest_path != None: 
            self.position_x = self.position_x + self.cell_grid.info.resolution*(goal_x-self.x)
            self.position_y = self.position_y + self.cell_grid.info.resolution*(goal_y-self.y)
            self.x = goal_x
            self.y = goal_y
            self.mav.follow_path(shortest_path)
            return True
        else:
            return False

    def droneBSA_loop(self):
        self.state = 0 
        surroundings = self.check_surroundings_2()
        self.update_cellmap(self.x,self.y,surroundings)
        self.state = 1

        while not surroundings[0] and not rospy.is_shutdown():  
            self.state = 1          
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
                self.check_for_sensor() 
                #rospy.loginfo("Spiral end detected")
                #rospy.loginfo("Attempting to backtrack")
                backtrack = self.drone_backtracking(self.x,self.y,self.cell_grid)
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
                self.check_for_sensor() 
                self.move()
                surroundings = self.check_surroundings_2()
                self.update_cellmap(self.x,self.y,surroundings)

            elif surroundings[0]:
                self.turn_right()
            else:
                self.check_for_sensor() 
                self.move()

    def droneMain(self):   
        self.initialize_cell_grid()
        for i in range(100):
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()
        self.mav.takeoff_and_keep_yaw(self.takeoff_alt)
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
        self.mav.hold(5)
        rospy.loginfo("Takeoff finished")     
        self.BSA_loop()  
        self.mav.land()
        self.mav._disarm() 
        rospy.loginfo(f"Seconds used: {(rospy.get_time() - self.seconds)}") 

    def droneMainTest(self):
        self.initialize_cell_grid()
        for i in range(100):
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()
        self.mav.takeoff_and_keep_yaw(self.takeoff_alt)
        self.mav.set_position_with_yaw(self.position_x,self.position_y,self.altitude)
        self.mav.hold(5)
        rospy.loginfo("Takeoff finished")  
        self.drone_go_front()
        self.drone_go_right()
        self.drone_go_back()
        self.drone_go_left()
        self.mav.land()
        self.mav._disarm()        

    
    

if __name__ == '__main__':
    rospy.init_node("BSA")
    bsa = BSA()
    bsa.droneMainTestmain()
    
