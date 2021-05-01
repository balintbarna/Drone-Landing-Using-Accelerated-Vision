from states.wait_for_control import WaitForControl
from states.search import Search

from states.target_common import *

class CenterTarget:
    def __init__(self, sm):
        self.filtered_distance = inf()
        self.dist_filter_ratio = rospy.get_param("dist_filter_ratio", 0.9)
        self.xy_max_err = rospy.get_param("xy_max_err", 0.2)

    def execute(self, sm):
        if not sm.mav.controllable():
            sm.set_state(WaitForControl(sm))
            return
        
        if is_data_old(sm):
            sm.set_state(Search(sm))
            return

        if (sm.landing_pose == None):
            return

        err = sm.landing_pose
        sm.landing_pose = None
        err.z = 0
        if (sm.get_filtered_distance(self, err) < self.xy_max_err):
            from states.approach_target import ApproachTarget
            sm.set_state(ApproachTarget(sm))
            return

        set_mav_pos_from_err(mav, err, False)
