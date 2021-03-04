#!/usr/bin/env python
# DNN sample code based on https://www.codespeedy.com/yolo-object-detection-from-image-with-opencv-and-python/

# import base libs
import numpy as np
from time import perf_counter

# import ROS and CV libraries
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2

class MainNode():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('object_detector', anonymous=True)
        self.setupNet()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageCallback, queue_size=1)
        self.pose_pub = rospy.Publisher("/landing_pos_error/raw", Point, queue_size=1)
        self.fps_pub_filtered = rospy.Publisher("/inferencing_fps/filtered", String, queue_size=1)
        self.fps_pub = rospy.Publisher("/inferencing_fps/raw", String, queue_size=1)
    
    def setupNet(self):
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

    def imageCallback(self, data):
        if (self.inferencing):
            return
        self.inferencing = True
        img = self.getCvImage(data)
        relative_pose = self.find_object(img)
        if (relative_pose != None):
            self.pose_pub.publish(relative_pose)
        self.inferencing = False
    
    def getCvImage(self, rosImage):
        try:
            img = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")
            return img
        except CvBridgeError as e:
            print(e)
        return None
    
    def find_object(self, img):
        start = perf_counter()
        height, width, channels = img.shape
        # rospy.logout("height: {}\nwidth: {}\nchannels: {}\n".format(height, width, channels))
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416,416), (0,0,0), True, crop=False)
        self.net.setInput(blob)
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
        cv2.imshow("Output",img)
        cv2.waitKey(1)

        return self.generate_pose(target_box, img.shape)
    
    def generate_pose(self, box, img_shape):
        if (box == None):
            return None
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
        p.z = 1 - size
        return p

def main():
    node = MainNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()