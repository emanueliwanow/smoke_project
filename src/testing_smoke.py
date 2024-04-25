#!/usr/bin/env python3

import rospy
import numpy as np
from mavros_msgs.srv import CommandLong

class SMOKE:
    def __init__(self):
        self.smoke = rospy.ServiceProxy("mavros/cmd/command", CommandLong)
    def actv_smoke(self,open):
        command = CommandLong()
        command.broadcast = False
        command.confirmation = 0
        command.command = 187
        command.param1 = open
        command.param2 = 0
        command.param3 = 0
        command.param4 = 0
        command.param5 = 0
        command.param6 = 0
        command.param7 = 0
        self.smoke.call(False,187,0,open,0,0,0,0,0,0)


if __name__ == "__main__":
    rospy.init_node("smoke_testing")
    smoke = SMOKE()
    smoke.actv_smoke(-1)
    smoke.actv_smoke(1)