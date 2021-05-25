import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

from mavros_driver.mav import Mav
# states
from states.startup import Startup

class StateMachine():
    def __init__(self):
        self._state = None
        self._state_pub = rospy.Publisher("/state_machine/state", String, queue_size=10)

        self.set_state(Startup(self))
        self.mav = Mav()
        self.landing_pose = Point()
        self.landing_pose = None
        self.landing_pose_stamp = rospy.Time.now()
        rospy.Subscriber("/landing_pos_error/local_frame", Point, self.landing_pose_callback)

    def landing_pose_callback(self, p = Point()):
        self.landing_pose = p
        self.landing_pose_stamp = rospy.Time.now()

    def get_name(self, obj):
        if hasattr(obj, "__name__"):
            return obj.__name__
        else:
            return str(obj)

    def set_state(self, new_state):
        old_state = self._state
        self._state = new_state
        new_name = self.get_name(new_state)
        old_name = self.get_name(old_state)
        time_stamp = rospy.Time.now()
        time_now = time_stamp.secs + time_stamp.nsecs/pow(10, 9)
        self._state_pub.publish(String("{} @ {}".format(new_name, time_now)))
        rospy.logout("New state: {} (was {})".format(new_name, old_name))

    def loop(self):
        if callable(self._state):
            self._state()
        elif hasattr(self._state, "execute"):
            self._state.execute(self)
        else:
            rospy.logerr("State is not callable and cannot execute: {}".format(self.get_name(self._state)))
