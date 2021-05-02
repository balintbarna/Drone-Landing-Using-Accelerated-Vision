class Landing:
    def __init__(self, sm):
        sm.mav.stop()
        sm.mav.auto_land()

    def execute(self, sm):
        if not sm.mav.state.armed:
            from states.passive import Passive
            sm.set_state(Passive(sm))