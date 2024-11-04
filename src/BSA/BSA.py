#!/usr/bin/env python3

import rospy
import numpy as np
import math
import time
import copy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose , PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray, Marker
from itertools import product
from astar.srv import astar_srv
from std_srvs.srv import Trigger

POSITIONS = [[10.2,17.1]]
# For SOS Lab
#[[10,5],[3.1, 34.6], [15,70.4], [25.1, 30.2],[12.5,30.0]]
# For warehouse
#[[1,1],[10.3,10.4],[15.2,18.1],[10.2,17.1]]


class BSA:
    def __init__(self,posX,posY):
        self.map_size = 300
        self.rate = rospy.Rate(60)
        self.costmapPose_x = 0
        self.costmapPose_y = 0
        self.state = 0 # 0:START 1:FORWARD 2:RIGHT 3:BACKWARDS 4:LEFT 5:END
        self.seconds = rospy.get_time()

        self.position_x = posX
        self.position_y = posY

        self.cell_origin = Pose()
        self.cell_resolution = 0.90
        self.cell_origin.position.x = -(((self.map_size/2)*self.cell_resolution)+(self.cell_resolution/2))+self.position_x
        self.cell_origin.position.y = -(((self.map_size/2)*self.cell_resolution)+(self.cell_resolution/2))+self.position_y
        self.cell_grid = OccupancyGrid()

        self.cartographer_grid = OccupancyGrid()

        self.debug_grid = OccupancyGrid()

        self.astar_path = None
        self.astar_oldPath = None
        self.astar_initial = PoseWithCovarianceStamped()
        self.astar_goal = PoseStamped()

        self.obstacle = 100
        self.visited = 50
        self.free = 0
        self.unknown = -1

        self.x,self.y = int(self.map_size/2),int(self.map_size/2)

        self.obstacle_th = 60


        self.astar_service = rospy.ServiceProxy('/astar_server', astar_srv)


        self.og_pub = rospy.Publisher('/scanned_area', OccupancyGrid, queue_size = 5)

        self.grid_sub = rospy.Subscriber('/map', OccupancyGrid, self.GridCallback)

        self.pose_pub = rospy.Publisher('/pose',PoseStamped,queue_size=5)
        self.pose  = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.pose.position.x = self.position_x
        self.pose.pose.position.y = self.position_y
        self.pose.pose.position.z = 0

        self.path_pub = rospy.Publisher('/path',Path,queue_size=5)
        self.path = Path()
        self.path.header.frame_id = 'map'

        self.astar_received = 0

        self.sensorDetection_srv = rospy.ServiceProxy('/sensor_detection', Trigger)
        self.sensor_positions = []

        self.distance_traveled = 0
        self.number_of_nodes = 0
        


    
    def initialize_cell_grid(self):
        self.cell_grid.header.frame_id = 'map'
        self.cell_grid.info.resolution = self.cell_resolution
        self.cell_grid.info.width = self.map_size
        self.cell_grid.info.height = self.map_size
        self.cell_grid.info.origin.position.x = self.cell_origin.position.x
        self.cell_grid.info.origin.position.y = self.cell_origin.position.y
        self.cell_grid.data = [self.unknown for i in range(self.map_size**2)]

    
    def GridCallback(self, grid_data):
        self.cartographer_grid = grid_data
    
    def AstarGetPath(self,initial_pose,goal):
        res = self.astar_service.call(initial_pose,goal)
        if not res.success:
            rospy.loginfo("No path found")
        return res.path,res.success
    
    def check_for_sensor(self):
        res = self.sensorDetection_srv.call()
        if res.success:
            flag = 0
            if len(self.sensor_positions) != 0:
                for i in range(len(self.sensor_positions)):
                    if abs(self.sensor_positions[i][0]-self.position_x)<0.9 and abs(self.sensor_positions[i][1]-self.position_y)<0.9:
                        flag = 1
            if flag == 0:
                #rospy.loginfo("New sensor detected")
                self.sensor_positions.append([self.position_x,self.position_y])
                #rospy.loginfo(f'Number of sensor detected: {len(self.sensor_positions)}')

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
    
    def is_in_gridmap(self, x, y, grid):
        if -1 < x < grid.info.width and -1 < y < grid.info.height:
            return True
        else:
            return False
    
    def get_cost_from_costmap_x_y(self, x, y,grid):
        if self.is_in_gridmap(x, y,grid):
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return grid.data[x+(y*grid.info.width)]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, grid.info.height, grid.info.width))

    def get_closest_cell_withAstar(self, x, y, grid, desired_cost = 0, max_radius=80):

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


    def update_cellmap(self,x,y,surroundings):
        self.cell_grid.data[x+((y)*self.cell_grid.info.width)] = self.visited
        #rospy.loginfo(f'Center in x:{x} , y:{y}')
        if self.state == 0:
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
            if not self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] == self.visited:
                if surroundings[2]:
                    self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] = self.free
            if not self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)]  == self.visited:
                if surroundings[3]:
                    self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.obstacle
                else:
                    self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] = self.free
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

    def check_surroundings_2(self):
        front,right,back,left = 0,0,0,0
        if self.check_for_obstacle_cell(self.cell_grid,self.cartographer_grid,1,0,self.position_x,self.position_y):
            front = 1
        if self.check_for_obstacle_cell(self.cell_grid,self.cartographer_grid,0,-1,self.position_x,self.position_y):
            right = 1
        if self.check_for_obstacle_cell(self.cell_grid,self.cartographer_grid,-1,0,self.position_x,self.position_y):
            back = 1
        if self.check_for_obstacle_cell(self.cell_grid,self.cartographer_grid,0,1,self.position_x,self.position_y):
            left = 1
        x,y = self.x,self.y
        if self.state == 0:
            return [front,right,back,left] 
        if self.state == 1:
            if self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] == self.visited:
                front = 1
            #if self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] == self.visited:
                #back = 1
            if self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] == self.visited:
                right = 1
            if self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] == self.visited:
                left = 1
            return [front,right,back,left]
        if self.state == 2:
            if self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] == self.visited:
                front = 1
            if self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] == self.visited:
                back = 1
            if self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] == self.visited:
                right = 1
            #if self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] == self.visited:
                #left = 1
            return [right,back,left,front]
        if self.state == 3:
            #if self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] == self.visited:
                #front = 1
            if self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] == self.visited:
                back = 1
            if self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] == self.visited:
                right = 1
            if self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] == self.visited:
                left = 1
            return [back,left,front,right]
        if self.state == 4:
            if self.cell_grid.data[x+1+((y)*self.cell_grid.info.width)] == self.visited:
                front = 1
            if self.cell_grid.data[x-1+((y)*self.cell_grid.info.width)] == self.visited:
                back = 1
            #if self.cell_grid.data[x+((y-1)*self.cell_grid.info.width)] == self.visited:
                #right = 1
            if self.cell_grid.data[x+((y+1)*self.cell_grid.info.width)] == self.visited:
                left = 1
            return [left,front,right,back]
        


    def go_to(self,x,y):
        n = 1
        dif_x = -self.pose.pose.position.x + x
        dif_y = -self.pose.pose.position.y + y
        dif_x_div = dif_x/n
        dif_y_div = dif_y/n


        for i in range(n):
            self.pose.pose.position.x += dif_x_div
            self.pose.pose.position.y += dif_y_div
            self.pose_pub.publish(self.pose)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x =  self.pose.pose.position.x
            pose.pose.position.y =  self.pose.pose.position.y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1
            self.path.poses.append(pose)
            self.path_pub.publish(self.path)
            self.rate.sleep()



        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose_pub.publish(self.pose)
        self.rate.sleep()


    def go_front(self):
        self.x += 1
        self.position_x += self.cell_grid.info.resolution
        self.go_to(self.position_x,self.position_y)
        
    
    def go_right(self):
        self.y -= 1
        self.position_y -= self.cell_grid.info.resolution
        self.go_to(self.position_x,self.position_y)
        
    def go_left(self):
        self.y += 1
        self.position_y += self.cell_grid.info.resolution
        self.go_to(self.position_x,self.position_y)
        
    def go_back(self):
        self.x -= 1
        self.position_x -= self.cell_grid.info.resolution
        self.go_to(self.position_x,self.position_y)
        
    def move(self):
        self.number_of_nodes+=1
        if self.state == 1:
            self.go_front()
        if self.state == 2:
            self.go_right()
        if self.state == 3:
            self.go_back()
        if self.state == 4:
            self.go_left()

    def turn_left(self):
        #rospy.loginfo("Turning left")
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
        #rospy.loginfo("Turning right")
        if self.state == 1:
            new_state = 2
        if self.state == 2:
            new_state = 3
        if self.state == 3:
            new_state = 4
        if self.state == 4:
            new_state = 1
        self.state = new_state

    def follow_path(self,path):
        for pose in path.poses:
            self.pose.pose.position.x = pose.pose.position.x
            self.pose.pose.position.y = pose.pose.position.y
            self.pose_pub.publish(self.pose)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x =  self.pose.pose.position.x
            pose.pose.position.y =  self.pose.pose.position.y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1
            self.path.poses.append(pose)
            self.path_pub.publish(self.path)
            self.rate.sleep()

    def backtracking(self,x,y,grid):
        # Backtracking without obstacle avoidance
        self.number_of_nodes+=1
        shortest_path,goal_x,goal_y = self.get_closest_cell_withAstar(x,y,grid)
        if shortest_path != None: 
            self.position_x = self.position_x + self.cell_grid.info.resolution*(goal_x-self.x)
            self.position_y = self.position_y + self.cell_grid.info.resolution*(goal_y-self.y)
            self.x = goal_x
            self.y = goal_y
            self.follow_path(shortest_path)
            return True
        else:
            return False


    def BSA_loop(self):
        self.state = 0 
        surroundings = self.check_surroundings_2()
        self.update_cellmap(self.x,self.y,surroundings)
        self.state = 1

        while not surroundings[0] and not rospy.is_shutdown():  
            self.state = 1  
            self.check_for_sensor()        
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
                backtrack = self.backtracking(self.x,self.y,self.cell_grid)
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

            


    def main(self):   
        self.initialize_cell_grid()
        for i in range(100):
            self.og_pub.publish(self.cell_grid)
            self.rate.sleep()     
        self.BSA_loop()  
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
        bsa = BSA(POSITIONS[i][0],POSITIONS[i][1])
        bsa.main()

