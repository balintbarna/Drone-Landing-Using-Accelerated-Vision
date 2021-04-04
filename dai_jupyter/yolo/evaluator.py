'''
NOTICE
This work is a derivative of https://github.com/Xilinx/Vitis-AI/blob/v1.1/mpsoc/vitis_ai_dnndk_samples/tf_yolov3_voc_py/tf_yolov3_voc.py
The license of the original work is included below
'''

'''
Copyright 2019 Xilinx Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''

import os
import cv2
import colorsys
import random
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from yolo.helper import *

class Evaluator:
    def __init__(self):
        # classes
        classes_path = "yolo/classes.txt"
        self.class_names = get_class(classes_path)
        # colors
        num_classes = len(self.class_names)
        hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
        colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(map(lambda x: 
                          (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), 
                          colors))
        random.seed(0)
        random.shuffle(self.colors)
        random.seed(None)
        # anchors
        anchor_list = [10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326]
        anchor_float = [float(x) for x in anchor_list]
        self.anchors = np.array(anchor_float).reshape(-1, 2)
        pass
    
    def evaluate(self, yolo_outputs, image_shape):
        score_thresh = 0.2
        anchor_mask = [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
        boxes = []
        box_scores = []
        input_shape = np.shape(yolo_outputs[0])[1 : 3]
        input_shape = np.array(input_shape)*32

        for i in range(len(yolo_outputs)):
            _boxes, _box_scores = boxes_and_scores(
                yolo_outputs[i], self.anchors[anchor_mask[i]], len(self.class_names), 
                input_shape, image_shape)
            boxes.append(_boxes)
            box_scores.append(_box_scores)
        boxes = np.concatenate(boxes, axis = 0)
        box_scores = np.concatenate(box_scores, axis = 0)

        mask = box_scores >= score_thresh
        boxes_ = []
        scores_ = []
        classes_ = []
        for c in range(len(self.class_names)):
            class_boxes_np = boxes[mask[:, c]]
            class_box_scores_np = box_scores[:, c]
            class_box_scores_np = class_box_scores_np[mask[:, c]]
            nms_index_np = nms_boxes(class_boxes_np, class_box_scores_np) 
            class_boxes_np = class_boxes_np[nms_index_np]
            class_box_scores_np = class_box_scores_np[nms_index_np]
            classes_np = np.ones_like(class_box_scores_np, dtype = np.int32) * c
            boxes_.append(class_boxes_np)
            scores_.append(class_box_scores_np)
            classes_.append(classes_np)
        boxes_ = np.concatenate(boxes_, axis = 0)
        scores_ = np.concatenate(scores_, axis = 0)
        classes_ = np.concatenate(classes_, axis = 0)

        return boxes_, scores_, classes_

    def draw_boxes(self, image, boxes, scores, classes):
        _, ax = plt.subplots(1)
        ax.imshow(image)
        image_h, image_w, _ = image.shape

        for i, bbox in enumerate(boxes):
            [top, left, bottom, right] = bbox
            width, height = right - left, bottom - top
            center_x, center_y = left + width*0.5, top + height*0.5
            score, class_index = scores[i], classes[i]
            label = '{}: {:.4f}'.format(self.class_names[class_index], score) 
            color = tuple([color/255 for color in self.colors[class_index]])
            ax.add_patch(Rectangle((left, top), width, height,
                                   edgecolor=color, facecolor='none'))
            ax.annotate(label, (center_x, center_y), color=color, weight='bold', 
                        fontsize=12, ha='center', va='center')
        return ax
    
    def pre_process(self, image, model_image_size):
        image_h, image_w, _ = image.shape

        if model_image_size != (None, None):
            assert model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(model_image_size)))
        else:
            new_image_size = (image_w - (image_w % 32), image_h - (image_h % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)
        return image_data