#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

class Transformer():
    def __init__(self):
        rospy.init_node('landing_pose_transformer_live', anonymous=True)
        rospy.Subscriber("/landing_pos_error/cam_frame/pid", Point, self.landing_pose_callback, queue_size=1)
        self.err_pub = rospy.Publisher("/landing_pos_error/drone_frame", Point, queue_size=1)
    
    def landing_pose_callback(self, msg = Point()):
        self.cam_frame_to_drone_frame(msg)
        self.err_pub.publish(msg)
    
    def cam_frame_to_drone_frame(self, p = Point()):
        x = -p.y
        y = -p.x
        z = -p.z
        p.x = x
        p.y = y
        p.z = z

def main():
    node = Transformer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()