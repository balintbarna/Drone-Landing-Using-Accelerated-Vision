#!/usr/bin/env python

# import ROS libraries
import rospy

# import local package
from mavros_driver.mav import Mav
from state_machine import StateMachine

class MainNode():
    def __init__(self):
        rospy.init_node('main', anonymous=True)
        self.mav = Mav()
        self.sm = StateMachine()
        self.sm.mav = self.mav

        self.stateMachineTimer = rospy.Timer(rospy.Duration(1/50), self.run_state)
    
    def run_state(self, timerEvent):
        self.sm.loop()

def main():
    node = MainNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()