import math

import rospy
from geometry_msgs.msg import Point, Pose

from mavros_driver.mav import Mav
from mavros_driver.message_tools import yaw_to_orientation

def inf():
    return float('inf')

class StateMachine():
    def __init__(self):
        self.state = None
        self.set_state(self.state_startup)
        self.mav = Mav()
        self.landing_pose = Point()
        self.landing_pose = None
        self.filtered_distance = inf()
        self.dist_filter_ratio = rospy.get_param("dist_filter_ratio", 0.9)
        self.xy_max_err = rospy.get_param("xy_max_err", 0.2)
        self.xyz_max_err_before_landing = rospy.get_param("xyz_max_err_before_landing", 0.1)
        self.takeoff_coords = []
        rospy.Subscriber("/landing_pos_error/local_frame", Point, self.landing_pose_callback)
    
    def landing_pose_callback(self, p = Point()):
        self.landing_pose = p
    
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
        if callable(self.state):
            self.state()
        else:
            rospy.logerr("State is not callable: {}".format(self.get_name(self.state)))

    def state_startup(self):
        if self.mav.connected():
            self.mav.start()
            self.set_state(self.state_wait_for_control)
    
    def state_wait_for_control(self):
        if self.mav.controllable():
            self.takeoff()
        else:
            self.mav.set_target_pose(self.mav.current_pose.pose)

    def takeoff(self):
        cp = self.mav.current_pose.pose.position
        p = Point(cp.x, cp.y, cp.z + rospy.get_param("starting_altitude", 1.0))
        o = yaw_to_orientation(0)
        home = Pose(p, o)
        self.mav.set_target_pose(home)
        if rospy.get_param("takeoff_test", True):
            # make a plus sign (+) movement to test basics
            d = rospy.get_param("takeoff_test_dist", 1.0)
            self.takeoff_coords = [
                Pose(Point(p.x + d, p.y, p.z), o),
                home,
                Pose(Point(p.x, p.y + d, p.z), o),
                home,
                Pose(Point(p.x - d, p.y, p.z), o),
                home,
                Pose(Point(p.x, p.y - d, p.z), o),
                home
            ]
        self.set_state(self.state_takeoff)

    def state_takeoff(self):
        if not self.mav.controllable():
            self.set_state(self.state_wait_for_control)

        if (self.mav.has_arrived()):
            if len(self.takeoff_coords) > 0:
                self.mav.set_target_pose(self.takeoff_coords.pop(0))
            else:
            self.set_state(self.state_inch_above_target)
    
    def state_passive(self):
        pass
    
    def state_loiter(self):
        self.mav.set_target_pose(self.mav.current_pose.pose)
    
    def state_inch_above_target(self):
        if not self.mav.controllable():
            self.set_state(self.state_wait_for_control)

        if (self.landing_pose == None):
            return
        err = self.landing_pose
        self.landing_pose = None
        err.z = 0
        if (self.get_filtered_distance(err) < self.xy_max_err):
            self.filtered_distance = inf()
            self.set_state(self.state_inch_lower_above_target)
            return
        self.set_mav_pos_from_err(err, False)
    
    def state_inch_lower_above_target(self):
        if not self.mav.controllable():
            self.set_state(self.state_wait_for_control)

        if (self.landing_pose == None):
            return
        err = self.landing_pose
        self.landing_pose = None
        if (self.get_filtered_distance(err) < self.xyz_max_err_before_landing):
            self.land()
            return
        z = err.z
        err.z = 0
        if (get_point_magnitude(err) > self.xy_max_err):
            self.filtered_distance = inf()
            self.set_state(self.state_inch_above_target)
            return
        err.z = z
        self.set_mav_pos_from_err(err)

    def land(self):
        self.filtered_distance = inf()
        self.mav.stop()
        self.mav.auto_land()
        self.set_state(self.state_landing)
    
    def state_landing(self):
        if not self.mav.state.armed:
            self.set_state(self.state_passive)

    def set_mav_pos_from_err(self, err, altitude = True):
        cp = self.mav.current_pose.pose.position
        z = cp.z - err.z
        if not altitude:
            z = self.mav.target_pose.position.z
        pos = Point(cp.x - err.x, cp.y - err.y, z)
        ori = yaw_to_orientation(0)
        self.mav.set_target_pose(Pose(pos, ori))

    def get_filtered_distance(self, p = Point):
        d = get_point_magnitude(p)
        if (self.filtered_distance == inf()):
            self.filtered_distance = d
        else:
            self.filtered_distance = self.filtered_distance * self.dist_filter_ratio + d * (1-self.dist_filter_ratio)
        return self.filtered_distance

def get_point_magnitude(p = Point()):
    return math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z)