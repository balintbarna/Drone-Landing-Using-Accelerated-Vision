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
from geometry_msgs.msg import TwistStamped, PoseStamped, Point, Pose
# own libs
from mavros_driver.message_tools import orientation_to_yaw, point_to_arr, create_setpoint_message_pos_ori, arr_to_point, create_setpoint_message_pose

class Mav():
    def __init__(self, namespace = "mavros"):
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.target_pose = Pose()
        self.state = State()

        mavros.set_namespace(namespace)

        self.maxdist = rospy.get_param("arrive_maxdist", 0.5) # m
        self.maxang = rospy.get_param("arrive_maxang", 0.1)
        self.maxvel = rospy.get_param("arrive_maxvel", 0.1) # m/s
        self.maxangvel = rospy.get_param("arrive_maxangvel", 0.05)
        self.target_maxdist = rospy.get_param("target_maxdist", 2.)
        self.pose_rate = rospy.get_param("pose_rate", 50.)

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
        if rospy.is_shutdown():
            self.stop()
            return
        if rospy.get_param("auto_arm_offboard", False):
            self.arm_and_offboard()
        self.publish_target_pose()

    def _state_callback(self, topic):
        self.state = topic

    def _local_position_callback(self, topic):
        self.current_pose = topic

    def _local_velocity_callback(self, topic):
        self.current_velocity = topic
    
    def arm_and_offboard(self):
        if not self.state.armed:
            if self.set_arming(True):
                pass
        if not self.state.mode == State.MODE_PX4_OFFBOARD:
            self.set_mode(0, State.MODE_PX4_OFFBOARD)
    
    def publish_target_pose(self):
        self._setpoint_local_pub.publish(self.distance_limited_target())
    
    def distance_limited_target(self):
        point = self.target_pose.position
        current = point_to_arr(self.current_pose.pose.position)
        target_arr = point_to_arr(point)
        diff = target_arr - current
        diffsize = np.linalg.norm(diff)
        if diffsize < self.target_maxdist:
            return create_setpoint_message_pose(self.target_pose)
        ratio = diffsize / self.target_maxdist
        true_target = current + diff / ratio
        new_point = arr_to_point(true_target)
        return create_setpoint_message_pos_ori(new_point, self.target_pose.orientation)
    
    def has_arrived(self):
        posgood = self.get_pos_error() < self.maxdist
        anggood = self.get_yaw_error() < self.maxang
        velgood = self.get_velocity_abs() < self.maxvel
        angvelgood = self.get_ang_vel_abs() < self.maxangvel
        if posgood and anggood and velgood and angvelgood:
            return True
        return False
    
    def get_pos_error(self):
        sp = self.target_pose.position
        cp = self.current_pose.pose.position
        x = sp.x-cp.x
        y = sp.y-cp.y
        z = sp.z-cp.z
        dist = math.sqrt(x*x + y*y + z*z)
        return dist
    
    def get_yaw_error(self):
        sy = orientation_to_yaw(self.target_pose.orientation)
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

    def set_target_pose(self, pose = Pose()):
        self.target_pose = pose
    
    def set_target_pos(self, pos = Point()):
        pose = Pose(pos, self.target_pose.orientation)
        self.set_target_pose(pose)

    def set_target_yaw(self, yaw):
        pose = Pose(self.target_pose.position, yaw_to_orientation(yaw))
        self.set_target_pose(pose)
    
    def auto_land(self):
        self.set_mode(0, State.MODE_PX4_LAND)
    
    def start(self):
        self.timer = rospy.Timer(rospy.Duration(1 / float(self.pose_rate)), self._timer_callback)

    def stop(self):
        self.timer.shutdown()
    
    def connected(self):
        if not self.state.connected:
            return False
        pos = self.current_pose.pose.position
        ori = self.current_pose.pose.orientation
        size = abs(pos.x) + abs(pos.y) + abs(pos.z) + abs(ori.x) + abs(ori.y) + abs(ori.z) + abs(ori.w)
        return size > 0
        
    def controllable(self):
        if not self.connected():
            return False
        if not self.state.armed:
            return False
        if not self.state.mode == State.MODE_PX4_OFFBOARD:
            return False
        return True