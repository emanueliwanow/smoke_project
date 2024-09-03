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
    
    
  
    
    mav.takeoff_and_keep_yaw(takeoff_alt)
    mav.hold(4)
    rospy.loginfo("Takeoff finished")
    
    mav.set_position_with_yaw(0,0,altitude)   
    mav.hold(2) 

    mav.set_position_with_yaw(1,0,altitude)   
    mav.hold(2) 
    
    mav.set_position_with_yaw(1,1,altitude)   
    mav.hold(2) 

    mav.set_position_with_yaw(0,1,altitude)   
    mav.hold(2) 

    mav.set_position_with_yaw(0,0,altitude)   
    mav.hold(2) 
    
    
    
    mav.land()
    mav._disarm()
    
if __name__ == "__main__":
    go()
