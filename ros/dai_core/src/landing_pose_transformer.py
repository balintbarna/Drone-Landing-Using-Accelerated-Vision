#!/usr/bin/env python

from math import cos, sin, pow
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion

def orientation_to_yaw(orientation = Quaternion()):
    quat_tf = [orientation.x, orientation.y, orientation.z, orientation.w]
    yaw = euler_from_quaternion(quat_tf)[2]
    return yaw

class Transformer():
    def __init__(self):
        self.orientation = Quaternion()
        self.last_pub = None
        rospy.init_node('landing_pose_transformer', anonymous=True)
        rospy.Subscriber("/landing_pos_error/drone_frame", Point, self.landing_pose_callback, queue_size=1)
        rospy.Subscriber("/mavros/local_position/pose/pose/orientation", Quaternion, self.orientation_callback, queue_size=1)
        self.err_pub = rospy.Publisher("/landing_pos_error/local_frame", Point, queue_size=1)
        self.rate_pub = rospy.Publisher("/landing_pos_error/update_duration", String, queue_size=1)

    def orientation_callback(self, msg = Quaternion()):
        self.orientation = msg
    
    def landing_pose_callback(self, msg = Point()):
        self.drone_frame_to_local(msg)
        self.err_pub.publish(msg)
        now = rospy.Time.now()
        if self.last_pub is not None:
            time_elapsed = now - self.last_pub
            self.rate_pub.publish(String(str(time_elapsed.secs + time_elapsed.nsecs/pow(10, 9))))
        self.last_pub = now
    
    def drone_frame_to_local(self, p = Point()):
        o = self.orientation
        o_sum = abs(o.x) + abs(o.y) + abs(o.z) + abs(o.w)
        if not o_sum > 0:
            return
        yaw = orientation_to_yaw(o)
        c = cos(yaw)
        s = sin(yaw)
        x = c*p.x - s*p.y
        y = s*p.x + c*p.y
        p.x = x
        p.y = y
        pass

def main():
    node = Transformer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()