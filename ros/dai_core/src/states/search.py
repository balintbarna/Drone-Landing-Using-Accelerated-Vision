from random import *
import rospy
from geometry_msgs.msg import Point, Pose
from mavros_driver.message_tools import yaw_to_orientation

class Search:
    def __init__(self, sm):
        self.search_dist = rospy.get_param("search_dist", 0.9)
        sm.landing_pose = None

    def execute(self, sm):
        if not sm.mav.controllable():
            from states.wait_for_control import WaitForControl
            sm.set_state(WaitForControl(sm))
            return
        
        if sm.landing_pose is not None:
            from states.center_target import CenterTarget
            sm.set_state(CenterTarget(sm))
            return

        if sm.mav.has_arrived():
            h = sm.home.position
            p = Point(h.x + self.rd(), h.y + self.rd(), h.z)
            o = yaw_to_orientation(0)
            pose = Pose(p, o)
            sm.mav.set_target_pose(pose)

    def rd(self):
        return (random() - 0.5) * self.search_dist * 2
