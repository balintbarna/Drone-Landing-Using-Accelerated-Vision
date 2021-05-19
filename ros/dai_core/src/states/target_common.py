import math
import rospy
from geometry_msgs.msg import Point, Pose
from mavros_driver.message_tools import yaw_to_orientation

def inf():
    return float('inf')

def get_point_magnitude(p = Point()):
    return math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z)

def set_mav_pos_from_err(mav, err, home, altitude = True):
    cp = mav.current_pose.pose.position
    z = cp.z - err.z
    if not altitude:
        z = mav.target_pose.position.z
    pos = Point(cp.x - err.x, cp.y - err.y, z)
    ori = home.orientation
    mav.set_target_pose(Pose(pos, ori))

def get_filtered_distance(state, p = Point()):
    d = get_point_magnitude(p)
    if (state.filtered_distance == inf()):
        state.filtered_distance = d
    else:
        state.filtered_distance = state.filtered_distance * state.dist_filter_ratio + d * (1-state.dist_filter_ratio)
    return state.filtered_distance

def is_data_old(sm):
    now = rospy.Time.now()
    time_elapsed = now - sm.landing_pose_stamp
    time_d = time_elapsed.secs + time_elapsed.nsecs/pow(10, 9)
    return time_d > 1.0
