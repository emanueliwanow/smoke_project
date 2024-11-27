#!/usr/bin/env python3

import rospy
import numpy as np
from mavros_msgs.srv import CommandLong, CommandAck,CommandInt
from mavros_msgs.msg import OverrideRCIn,ActuatorControl

class SMOKE:
    def __init__(self):
        #self.smoke = rospy.ServiceProxy("mavros/cmd/command", CommandLong)
        #self.smoke_ack = rospy.ServiceProxy("mavros/cmd/command_ack", CommandAck)
        self.smoke_int = rospy.ServiceProxy("mavros/cmd/command_int", CommandInt)
        #self.rc_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size = 20)
        self.rate = rospy.Rate(30)
        
    def actv_smoke(self,open):
        # If open: 1 -> open , open: -1 -> closed
        command = CommandInt()
        if open >1:
            rospy.loginfo('Closing smoke spreader')
        else:
            rospy.loginfo('Opening smoke spreader')
        self.smoke_int.call(False,0,187,0,0,open,0,0,0,0,0,0)
        #self.smoke_ack.call(187,0,0,0)
    '''
    def actv_smoke2(self,open):
        command = CommandLong()
        self.smoke.call(False,183,0,1,1900,0,0,0,0,0)
        #self.smoke.call(False,183,0,2,2000,0,0,0,0,0)
        #self.smoke.call(False,183,0,3,2000,0,0,0,0,0)
        #self.smoke.call(False,183,0,4,2000,0,0,0,0,0)
        #self.smoke.call(False,183,0,5,2000,0,0,0,0,0)
        #self.smoke.call(False,183,0,6,2000,0,0,0,0,0)
        #self.smoke.call(False,183,0,7,2000,0,0,0,0,0)
    def actv_smoke3(self,open):
        command = CommandLong()
        self.smoke_int.call(False,1,187,0,0,open,0,0,0,0,0,0)
        

    def OverrideRC(self):
        command = OverrideRCIn()
        command.channels = [65535,65535,65535,65535,65535,65535,2000,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535]
        while not rospy.is_shutdown():
            self.rc_pub.publish(command)
            #self.rate.sleep()
    def Servo_control(self):

        pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)


        msg_out = ActuatorControl()
        msg_out.group_mix = 2 # Use group 2 (auxilary controls)
        msg_out.header.frame_id = 'base_link'
        msg_out.controls = [1.0, -1.0, 1.0, -1.0, 0.0, 0.0, 0.0, 0.0]

        while not rospy.is_shutdown():
            msg_out.controls = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]

            rospy.loginfo("Set servos high")

            msg_out.header.stamp = rospy.Time.now()
            pub.publish(msg_out)
            self.rate.sleep()
    '''

if __name__ == "__main__":
    rospy.init_node("smoke_testing")
    
    smoke = SMOKE()
    smoke.actv_smoke(-1)
    rospy.sleep(2)
    smoke.actv_smoke(1)
    """
    smoke = SMOKE()
    smoke.OverrideRC()
    """
   
    