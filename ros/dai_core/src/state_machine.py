import math

import rospy
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String

from mavros_driver.mav import Mav
from mavros_driver.message_tools import yaw_to_orientation
# states
from states.passive import Passive
from states.startup import Startup
from states.wait_for_control import WaitForControl
from states.takeoff import TakeOff
from states.center_target import CenterTarget

from states.target_common import *

class StateMachine():
    def __init__(self):
        self._state = None
        self._state_pub = rospy.Publisher("/state_machine/state", String, queue_size=10)

        self.set_state(Startup(self))
        self.mav = Mav()
        self.landing_pose = Point()
        self.landing_pose = None
        self.landing_pose_stamp = rospy.Time.now()
        rospy.Subscriber("/landing_pos_error/local_frame", Point, self.landing_pose_callback)

    def landing_pose_callback(self, p = Point()):
        self.landing_pose = p
        self.landing_pose_stamp = rospy.Time.now()

    def get_name(self, obj):
        if hasattr(obj, "__name__"):
            return obj.__name__
        else:
            return str(obj)

    def set_state(self, new_state):
        old_state = self._state
        self._state = new_state
        self._state_pub.publish(String(self.get_name(new_state)))
        rospy.logout("New state: {} (was {})".format(self.get_name(new_state), self.get_name(old_state)))

    def loop(self):
        if callable(self._state):
            self._state()
        elif hasattr(self._state, "execute"):
            self._state.execute(self)
        else:
            rospy.logerr("State is not callable and cannot execute: {}".format(self.get_name(self._state)))
