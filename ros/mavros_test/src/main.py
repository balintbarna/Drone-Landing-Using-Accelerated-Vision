#!/usr/bin/env python3

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

class MainNode():
    def __init__(self):
        rospy.init_node('main', anonymous=True)
        self.rate = rospy.Rate(20)
        self.mav = Mav()
    
    def loop(self):
        while (True):
            rospy.loginfo(self.mav.UAV_state)
            self.rate.sleep()

def main():
    node = MainNode()
    node.loop()

def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    main()