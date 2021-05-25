#!/usr/bin/env python
'''
NOTICE
This work is a derivative of https://www.codespeedy.com/yolo-object-detection-from-image-with-opencv-and-python/
The setup_net and find_object functions contain elements from the above linked sample code
'''

# import base libs
import numpy as np
import math
from threading import Condition, Thread
from time import perf_counter, sleep

# import ROS and CV libraries
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ObjectDetector(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.bridge = CvBridge()
        rospy.init_node('object_detector', anonymous=True)
        self.setup_net()
        self.image = Image()
        self.image = None
        self.new_img_recv = Condition()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)
        self.pose_pub = rospy.Publisher("/landing_pos_error/cam_frame/raw", Point, queue_size=1)
        self.fps_pub_filtered = rospy.Publisher("/inferencing_fps/filtered", String, queue_size=1)
        self.fps_pub = rospy.Publisher("/inferencing_fps/raw", String, queue_size=1)
        self.marked_img_pub = rospy.Publisher("/marked_img", Image, queue_size=1)
        self.extra_delay = rospy.get_param("extra_inferencing_delay", 0.)
        self.start()
    
    def setup_net(self):
        params_path = rospy.get_param("pkg_path") + "/resources/"
        self.net = cv2.dnn.readNet(params_path + "yolov3.weights", params_path + "yolov3.cfg")
        enable_gpu = rospy.get_param("gpu", False)
        rospy.logout("GPU acceleration set to: {}".format(enable_gpu))
        if (enable_gpu):
            rospy.logout("Configuring GPU execution")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        #To load all objects that have to be detected
        self.classes = []
        with open(params_path + "coco.names","r") as f:
            read = f.readlines()
        for i in range(len(read)):
            self.classes.append(read[i].strip("\n"))
        #Defining layer names
        layer_names = self.net.getLayerNames()
        self.output_layers = []
        for i in self.net.getUnconnectedOutLayers():
            self.output_layers.append(layer_names[i[0]-1])
        self.inferencing = False
        self.filtered_fps = 0
        self.fps_filter_ratio = rospy.get_param("fps_filter_ratio", 0.9)
        self.target_label = rospy.get_param("target_label", "stop sign")
        rospy.logout("Object detector initialized")

    def image_callback(self, data):
        self.image = data
        with self.new_img_recv:
            self.new_img_recv.notify_all()
    
    def run(self):
        while not rospy.is_shutdown():
            self.process_image()
            with self.new_img_recv:
                self.new_img_recv.wait()
    
    def process_image(self):
        if (self.inferencing):
            return
        try:
            self.inferencing = True
            if (self.image is None):
                raise Exception("ROS Image is None")
            ros_img = self.image
            self.image = None
            img = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
            relative_pose = self.find_object(img)
            self.pose_pub.publish(relative_pose)
        except Exception as e:
            pass
        except CvBridgeError as e:
            print(e)
        finally:
            self.inferencing = False
    
    def find_object(self, img):
        if (img is None):
            raise Exception("img is None")
        start = perf_counter()
        height, width, channels = img.shape
        # rospy.logout("height: {}\nwidth: {}\nchannels: {}\n".format(height, width, channels))
        blob = cv2.dnn.blobFromImage(img, 1.0 / 256., (416,416), (0,0,0), True, crop=False)
        self.net.setInput(blob)
        if self.extra_delay > 0:
            sleep(self.extra_delay)
        outs=self.net.forward(self.output_layers)
        class_ids=[]
        confidences=[]
        boxes=[]
        for output in outs:
            for detection in output:
                #Detecting confidence in 3 steps
                scores = detection[5:]            #1
                class_id = np.argmax(scores)      #2
                confidence = scores[class_id]     #3
                if confidence > 0.5: #Means if the object is detected
                    center_x = int(detection[0]*width)
                    center_y = int(detection[1]*height)
                    w = int(detection[2]*width)
                    h = int(detection[3]*height)
                    #Drawing a rectangle
                    x = int(center_x-w/2) # top left value
                    y = int(center_y-h/2) # top left value
                    boxes.append([x,y,w,h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
        #Removing Double Boxes
        indices_list = cv2.dnn.NMSBoxes(boxes, confidences, 0.3, 0.4)
        indices = np.array(indices_list).flatten()
        target_box = None
        for i in indices:
            x, y, w, h = boxes[i]
            label = self.classes[class_ids[i]]  # name of the objects
            if (label == self.target_label):
                target_box = boxes[i]
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, label, (x, y), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
           
        end = perf_counter()
        fps = (1/(end-start))
        self.filtered_fps = self.filtered_fps * self.fps_filter_ratio + fps * (1-self.fps_filter_ratio)
        self.fps_pub.publish(String("{}/s".format(fps)))
        self.fps_pub_filtered.publish(String("{}/s".format(self.filtered_fps)))
        self.marked_img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

        return self.generate_pose(target_box, img.shape)
    
    def generate_pose(self, box, img_shape):
        if (box is None):
            raise Exception("box is None")
        height, width, channels = img_shape
        x, y, w, h = box
        x /= width
        y /= height
        w /= width
        h /= height
        size = w*h
        p = Point()
        p.x = x*2 + w - 1
        p.y = y*2 + h - 1
        p.z = 1 - math.pow(size, 1/4)
        return p

def main():
    node = ObjectDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cond = node.new_img_recv
        with cond:
            cond.notify_all()
        node.join()

if __name__ == '__main__':
    main()
