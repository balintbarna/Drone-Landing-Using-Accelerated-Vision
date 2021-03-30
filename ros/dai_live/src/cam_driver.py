#!/usr/bin/env python

from threading import Condition, Thread

# import ROS and CV libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CamDriver(Thread):
    def __init__(self):
        Thread.__init__(self)
        rospy.init_node('cam_driver', anonymous=True)
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size=1)
        self.cap = cv2.VideoCapture(0)
        self.start()
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                ret, img = self.cap.read()
                # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                imgmsg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                # imgmsg = self.bridge.cv2_to_imgmsg(gray, "mono8")
                self.img_pub.publish(imgmsg)
            except Exception as e:
                rospy.logout(e)
        rospy.logout("Shutting down camera driver...")
    
    def close(self):
        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    node = CamDriver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()

if __name__ == '__main__':
    main()
