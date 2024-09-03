#!/usr/bin/env python3

import rospy
from mavbase.MAV import MAV
import numpy as np
import math

def go():

    rospy.init_node("mav_test")
    mav = MAV("1")

    altitude = 0.6
    takeoff_alt = altitude
    start_x = 0
    radius = 0.5
    start_y = radius
    start_yaw = -math.pi/2
    circle_discretization = 20
    time_step = 100
    angle_step = 2*math.pi/circle_discretization
    center_circle_x = 0
    center_circle_y = 0
    
  
    
    mav.takeoff_and_keep_yaw(takeoff_alt)
    mav.hold(4)
    rospy.loginfo("Takeoff finished")
    
    mav.set_position_with_yaw(start_x,start_y,altitude,start_yaw)   
    mav.hold(2) 
    
   
    
    rospy.loginfo("Starting circle")
      
    
    for step in range(circle_discretization+1):
    	
        angle = -start_yaw+ (step*angle_step)
        goal_x = (center_circle_x-math.cos(angle))*radius
        goal_y = (center_circle_y+math.sin(angle))*radius
        rospy.loginfo("Updating position")
        goal_yaw = -angle
        mav.set_position_with_yaw(goal_x,goal_y,altitude,goal_yaw)
        
    
   
    rospy.loginfo("Circle finished")
    """
    
    for j in range(circle_discretization*time_step):
    	k = j//time_step
    	start_yaw = angle_step * k
    	mav.set_position_with_yaw(start_x, start_y, altitude, start_yaw)

    """
    
    
    mav.land()
    mav._disarm()
    
if __name__ == "__main__":
    go()
