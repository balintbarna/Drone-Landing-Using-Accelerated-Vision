#!/usr/bin/env python

# import base libs
import numpy as np
import sys
import signal
import math

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose

# import files in package
from mavros_driver.mav import Mav
from state_machine import StateMachine

class MainNode():
    def __init__(self):
        rospy.init_node('main', anonymous=True)
        self.mav = Mav()
        self.sm = StateMachine()
        self.sm.mav = self.mav

        self.stateMachineTimer = rospy.Timer(rospy.Duration(1/50), self.run_state)
    
    def run_state(self, timerEvent):
        self.sm.loop()

def main():
    node = MainNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()