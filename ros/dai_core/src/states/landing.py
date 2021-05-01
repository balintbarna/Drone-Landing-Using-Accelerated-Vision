class Landing:
    def __init__(self, sm):
        self.filtered_distance = inf()
        self.mav.stop()
        self.mav.auto_land()

    def execute(self, sm):
        if not self.mav.state.armed:
            self.set_state(Passive(self))