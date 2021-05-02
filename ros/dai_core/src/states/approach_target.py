import rospy
from states.target_common import *

class ApproachTarget:
    def __init__(self, sm):
        self.filtered_distance = inf()
        self.dist_filter_ratio = rospy.get_param("dist_filter_ratio", 0.9)
        self.xy_max_err = rospy.get_param("xy_max_err", 0.2)
        self.xyz_max_err_before_landing = rospy.get_param("xyz_max_err_before_landing", 0.1)

    def execute(self, sm):
        if not sm.mav.controllable():
            from states.wait_for_control import WaitForControl
            sm.set_state(WaitForControl(sm))
            return
        
        if is_data_old(sm):
            from states.search import Search
            sm.set_state(Search(sm))
            return

        if (sm.landing_pose == None):
            return

        err = sm.landing_pose
        sm.landing_pose = None
        if (get_filtered_distance(self, err) < self.xyz_max_err_before_landing):
            from states.landing import Landing
            sm.set_state(Landing(sm))
            return

        z = err.z
        err.z = 0
        if (get_point_magnitude(err) > self.xy_max_err):
            from states.center_target import CenterTarget
            sm.set_state(CenterTarget(sm))
            return

        err.z = z
        set_mav_pos_from_err(sm.mav, err)
