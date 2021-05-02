class WaitForControl: 
    def __init__(self, sm):
        pass
   
    def execute(self, sm):
        if sm.mav.controllable():
            from states.takeoff import TakeOff
            sm.set_state(TakeOff(sm))
        else:
            sm.mav.set_target_pose(sm.mav.current_pose.pose)