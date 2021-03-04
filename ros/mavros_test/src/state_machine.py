from simple_pid import PID

import rospy
from geometry_msgs.msg import Point

from mavros_driver.mav import Mav
from mavros_driver.message_tools import create_setpoint_message_pose

class PointPid():
    def __init__(self):
        px = 1
        py = px
        pz = px / 2
        self.x = PID(Kp = px, Kd = px/10)
        self.y = PID(Kp = py, Kd = py/10)
        self.z = PID(Kp = pz, Kd = pz/10)

class StateMachine():
    def __init__(self):
        self.state = None
        self.set_state(self.startup)
        self.mav = Mav()
        self.landing_pose = Point()
        self.landing_pose = None
        self.pos_pid = PointPid()
        rospy.Subscriber("/landing_pos_error/raw", Point, self.landing_pose_callback)
        self.err_pub = rospy.Publisher("/landing_pos_error/transformed", Point, queue_size=1)
    
    def landing_pose_callback(self, p = Point()):
        tfd = self.cam_frame_to_local(p)
        nu = Point()
        nu.x = self.pos_pid.x(tfd.x)
        nu.y = self.pos_pid.y(tfd.y)
        nu.z = self.pos_pid.z(tfd.z)
        self.err_pub.publish(nu)
        self.landing_pose = nu
    
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
            new_target.pose.position.z = 10
            self.mav.set_target_pose(new_target)
            self.mav.start()
            self.set_state(self.takeoff)


    def takeoff(self):
        if (self.mav.has_arrived()):
            self.set_state(self.inch_above_target)
    
    def loiter(self):
        pass
    
    def inch_above_target(self):
        if (self.landing_pose == None):
            return
        err = self.landing_pose
        self.landing_pose = None
        new_target = create_setpoint_message_pose(self.mav.current_pose.pose)
        cpos = new_target.pose.position
        cpos.x -= err.x
        cpos.y -= err.y
        self.mav.set_target_pose(new_target)
        if (abs(cpos.x) < 0.01):
            self.set_state(self.loiter)
    
    def cam_frame_to_local(self, pos_in_cam = Point()):
        nu = Point()
        nu.x = -pos_in_cam.y
        nu.y = -pos_in_cam.x
        nu.z = -pos_in_cam.z
        return nu

