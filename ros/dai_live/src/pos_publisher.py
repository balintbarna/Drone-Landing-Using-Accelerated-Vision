#!/usr/bin/env python

from threading import Condition, Thread
import sys
import struct
import math
import time
# import ROS and zmq libraries
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import zmq

class ZmqTest(Thread):
    def __init__(self):
        Thread.__init__(self)
        rospy.init_node('zmq_test', anonymous=True)
        self.pose_pub = rospy.Publisher("/landing_pos_error/cam_frame/raw", Point, queue_size=1)

        ctx = zmq.Context()
        self.sub = ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.connect("tcp://localhost:5555")
        self.pub = ctx.socket(zmq.PUB)
        self.pub.setsockopt(zmq.LINGER, 0)
        self.pub.connect("tcp://localhost:5556")
        self.start()
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                messages = self.sub.recv_multipart()
                self.on_recv(messages)
            except KeyboardInterrupt:
                print("Error while receiving message")
                break
        rospy.logout("Pos reading thread terminated")
    
    def on_recv(self, messages):
        [top, left, bottom, right] = [self.deserialize_float_msg(m) for m in messages]
        w = right - left
        h = bottom - top
        size = w*h
        p = Point()
        p.x = (left + right) - 1
        p.y = (top + bottom) - 1
        p.z = 1 - math.pow(size, 1/4.)
        self.pose_pub.publish(p)
    
    def deserialize_float_msg(self, msg):
        s = sys.getsizeof(msg)
        t = 'd' if s is 45 else 'f' if s is 41 else 'err'
        value = struct.unpack(t, msg)[0]
        return value
    
    def close(self):
        self.pub.send(b"TERMINATE")
        self.join(5)
        time.sleep(1)
        self.sub.close()
        self.pub.close()

def main():
    node = ZmqTest()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()

if __name__ == '__main__':
    main()
