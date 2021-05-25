class Startup:
    def __init__(self, sm):
        pass

    def execute(self, sm):
        if sm.mav.connected():
            sm.mav.start()
            from states.wait_for_control import WaitForControl
            sm.set_state(WaitForControl(sm))
