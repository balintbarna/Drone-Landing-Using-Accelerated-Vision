from random import *
import rospy
from states.wait_for_control import WaitForControl
from states.center_target import CenterTarget

class Search:
    def __init__(self, sm):
        self.search_dist = rospy.get_param("search_dist", 0.9)

    def execute(self, sm):
        if not sm.mav.controllable():
            sm.set_state(WaitForControl(sm))
            return
        
        if sm.landing_pose is not None:
            sm.set_state(CenterTarget(sm))
            return

        if sm.mav.has_arrived():
            h = sm.home
            p = Point(h.x + self.rd(), h.y + self.rd(), h.z + self.rd())
            o = yaw_to_orientation(0)
            pose = Pose(p, o)
            sm.mav.set_target_pose(pose)

    def rd(self):
        return (random() - 1) * self.search_dist * 2
