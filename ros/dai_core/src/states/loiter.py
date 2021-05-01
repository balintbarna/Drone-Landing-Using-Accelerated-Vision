class Loiter:
    def __init__(self, sm):
        pass

    def execute(self, sm):
        sm.mav.set_target_pose(sm.mav.current_pose.pose)