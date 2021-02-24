# python libs
import math
import numpy as np
# ROS libs
import rospy
import mavros
import mavros.setpoint
import mavros.command
from mavros_msgs.msg import State
import mavros_msgs.srv
from geometry_msgs.msg import TwistStamped, PoseStamped, Point
# own libs
from mavros_driver.message_tools import create_setpoint_message_pos_yaw, orientation_to_yaw, point_to_arr, create_setpoint_message_pos_ori, arr_to_point

class Mav():
    def __init__(self, namespace = "mavros"):
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.target_pose = PoseStamped()
        self.state = State()

        mavros.set_namespace(namespace)

        # setup subscriber
        self._state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self._state_callback)
        self._local_position_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, self._local_position_callback)
        self._local_velocity_sub = rospy.Subscriber(mavros.get_topic('local_position', 'velocity_body'), TwistStamped, self._local_velocity_callback)

        # setup publisher
        self._setpoint_local_pub = mavros.setpoint.get_pub_position_local(queue_size=10)

        # setup service
        self.set_arming = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), mavros_msgs.srv.CommandBool)
        self.set_mode = rospy.ServiceProxy(mavros.get_topic('set_mode'), mavros_msgs.srv.SetMode)
    
    def _timer_callback(self, timerEvent):
        self.arm_and_offboard()
        self.publish_target_pose()

    def _state_callback(self, topic):
        self.state = topic

    def _local_position_callback(self, topic):
        self.current_pose = topic

    def _local_velocity_callback(self, topic):
        self.current_velocity = topic
    
    def arm_and_offboard(self):
        if self.state.mode != "OFFBOARD":
            self.set_mode(0, 'OFFBOARD')
        if not self.state.armed:
            if self.set_arming(True):
                pass
    
    def publish_target_pose(self):
        # self._setpoint_local_pub.publish(self.target_pose)
        self._setpoint_local_pub.publish(self.distance_limited_target())
        pass
    
    def distance_limited_target(self):
        point = self.target_pose.pose.position
        current = point_to_arr(self.current_pose.pose.position)
        target_arr = point_to_arr(point)
        diff = target_arr - current
        diffsize = np.linalg.norm(diff)
        max_dist = 2
        if diffsize < max_dist:
            return self.target_pose
        ratio = diffsize / max_dist
        true_target = current + diff / ratio
        new_point = arr_to_point(true_target)
        return create_setpoint_message_pos_ori(new_point, self.target_pose.pose.orientation)
    
    def has_arrived(self):
        maxdist = 0.5 # m
        maxang = 0.1 # 6 deg in rad
        max_vel = 0.2 # m/s
        max_angvel = 0.05
        posgood = self.get_pos_error() < maxdist
        anggood = self.get_yaw_error() < maxang
        velgood = self.get_velocity_abs() < max_vel
        angvelgood = self.get_ang_vel_abs() < max_angvel
        if posgood and anggood and velgood and angvelgood:
            return True
        return False
    
    def get_pos_error(self):
        sp = self.target_pose.pose.position
        cp = self.current_pose.pose.position
        x = sp.x-cp.x
        y = sp.y-cp.y
        z = sp.z-cp.z
        dist = math.sqrt(x*x + y*y + z*z)
        return dist
    
    def get_yaw_error(self):
        sy = orientation_to_yaw(self.target_pose.pose.orientation)
        cy = orientation_to_yaw(self.current_pose.pose.orientation)
        yaw = abs(sy - cy)
        return yaw
    
    def get_velocity_abs(self):
        vel = self.current_velocity.twist.linear
        x = vel.x
        y = vel.y
        z = vel.z
        norm = math.sqrt(x*x + y*y + z*z)
        return norm
    
    def get_ang_vel_abs(self):
        vel = self.current_velocity.twist.angular
        x = vel.x
        y = vel.y
        z = vel.z
        norm = math.sqrt(x*x + y*y + z*z)
        return norm

    def set_target_pose(self, pose = PoseStamped()):
        self.target_pose = pose
    
    def set_target_pos(self, pos = Point()):
        yaw = orientation_to_yaw(self.target_pose.pose.orientation)
        pose = create_setpoint_message_pos_yaw(pos, yaw)
        self.set_target_pose(pose)

    def set_target_yaw(self, yaw):
        pos = self.target_pose.pose.position
        pose = create_setpoint_message_pos_yaw(pos, yaw)
        self.set_target_pose(pose)
    
    def start(self):
        self.timer = rospy.Timer(rospy.Duration(1/20), self._timer_callback)

    def is_ready(self):
        return self.state.connected