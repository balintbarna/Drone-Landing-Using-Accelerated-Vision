from geometry_msgs.msg import Point, Quaternion, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from mavros.setpoint import PoseStamped, Header
import numpy as np

def yaw_to_orientation(yaw):
    quat_tf = quaternion_from_euler(0, 0, yaw)
    ori = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
    return ori

def orientation_to_yaw(orientation = Quaternion()):
    quat_tf = [orientation.x, orientation.y, orientation.z, orientation.w]
    yaw = euler_from_quaternion(quat_tf)[2]
    return yaw

def pose_from_xyz_yaw(x, y, z, yaw = 0):
    pos = Point(x,y,z)
    ori = yaw_to_orientation(yaw)
    return Pose(pos, ori)

def create_setpoint_message_pos_ori(pos = Point(), ori = Quaternion()):
    pose = Pose(pos, ori)
    return create_setpoint_message_pose(pose)

def create_setpoint_message_pose(pose = Pose()):
    setpoint_msg = PoseStamped(
        header = Header(
            frame_id = "map",
            stamp = rospy.Time.now()
        ),
        pose = pose
    )
    return setpoint_msg

def point_to_arr(point = Point()):
    arr = np.array([point.x, point.y, point.z])
    return arr

def arr_to_point(arr):
    return Point(arr[0], arr[1], arr[2])