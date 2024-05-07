#!/usr/bin/env python3

import rospy
from mavbase.MAV import MAV
import numpy as np

def go():

    rospy.init_node("mav_test")
    mav = MAV("1")

    takeoff_alt = 1
    
    altitude = 1

    mav.takeoff(takeoff_alt)
    rospy.loginfo("Takeoff finished")
    
    mav.set_position(0,0,altitude)    
    while not mav.chegou():
        mav.set_position(0,0,altitude)
    
    
    
    rospy.loginfo("On hold")
    mav.hold(5)
    
    #mav.set_altitude(1)
    mav.land()
    mav._disarm()
    
if __name__ == "__main__":
    go()