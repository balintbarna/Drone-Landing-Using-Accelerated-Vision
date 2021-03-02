#!/usr/bin/env python

# import base libs
import numpy as np
import sys
import signal

# import ROS libraries
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped, Vector3, Vector3Stamped, Point, Quaternion, Pose
from cv_bridge import CvBridge, CvBridgeError
import cv2

class MainNode():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('object_detector', anonymous=True)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageCallback, queue_size=1)
        self.pose_pub = rospy.Publisher("/landing_pose", PoseStamped, queue_size=1)
    
    def imageCallback(self, data):
        img = self.getCvImage(data)
        relative_pose = self.find_object(img)
        self.pose_pub.publish(relative_pose)
    
    def getCvImage(self, rosImage):
        try:
            img = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
            return img
        except CvBridgeError as e:
            print(e)
        return None
    
    def find_object(self, img):
        pose = PoseStamped()
        return pose

def main():
    node = MainNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()