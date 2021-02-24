import rospy
from geometry_msgs.msg import Point

from mavros_driver.mav import Mav
from mavros_driver.message_tools import create_setpoint_message_pose

class StateMachine():
    def __init__(self):
        self.state = None
        self.set_state(self.startup)
        self.mav = Mav()
    
    def get_name(self, obj):
        if hasattr(obj, "__name__"):
            return obj.__name__
        else:
            return str(obj)

    def set_state(self, new_state):
        old_state = self.state
        self.state = new_state
        rospy.logout("New state: {} (was {})".format(self.get_name(new_state), self.get_name(old_state)))

    def loop(self):
        if (callable(self.state)):
            self.state()
        else:
            rospy.logerr("State is not callable: {}".format(self.get_name(self.state)))

    def startup(self):
        if (self.mav.is_ready()):
            new_target = create_setpoint_message_pose(self.mav.current_pose.pose)
            new_target.pose.position.z = new_target.pose.position.z + 5
            self.mav.set_target_pose(new_target)
            self.mav.start()
            self.set_state(self.takeoff)


    def takeoff(self):
        if (self.mav.has_arrived()):
            self.set_state(self.loiter)
    
    def loiter(self):
        pass
