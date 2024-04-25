#!/usr/bin/env python3

import rospy
import numpy as np
from mavros_msgs.srv import CommandLong

class SMOKE:
    def __init__(self):
        self.smoke = rospy.ServiceProxy("mavros/cmd/command", CommandLong)
    def actv_smoke(self,open):
        command = CommandLong()
        command.command = 187
        command.param1 = open
        self.smoke.call(command)


if __name__ == "__main__":
    rospy.init_node("smoke_testing")
    smoke = SMOKE()
    smoke.actv_smoke(-1)
    smoke.actv_smoke(1)