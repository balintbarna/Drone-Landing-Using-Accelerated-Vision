'''
NOTICE
This work is a derivative of https://github.com/Xilinx/Vitis-AI/blob/v1.1/mpsoc/vitis_ai_dnndk_samples/tf_yolov3_voc_py/tf_yolov3_voc.py
and https://github.com/Xilinx/DPU-PYNQ/blob/v1.2.0/pynq_dpu/edge/notebooks/dpu_yolo_v3.ipynb
The license of the original work(s) is included below
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

class Evaluator:
    def __init__(self):
        self.class_names = self.read_class_list("yolo/classes.txt")
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
        self.anchor_mask = [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
        pass

    def read_class_list(self, path):
        with open(path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names
    
    def pre_process(self, image, model_image_size):
        image_h, image_w, _ = image.shape

        if model_image_size != (None, None):
            assert model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = self.letterbox_image(image, tuple(reversed(model_image_size)))
        else:
            new_image_size = (image_w - (image_w % 32), image_h - (image_h % 32))
            boxed_image = self.letterbox_image(image, new_image_size)
        image_data = boxed_image
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)
        return image_data

    def letterbox_image(self, image, size, show = False):
        '''resize image with unchanged aspect ratio using padding'''
        ih, iw, _ = image.shape
        w, h = size
        scale = min(w/iw, h/ih)
        
        nw = int(iw*scale)
        nh = int(ih*scale)

        image = cv2.resize(image, (nw,nh), interpolation=cv2.INTER_LINEAR)
        new_image = np.ones((h,w,3), dtype='float32') * 128
        h_start = (h-nh)//2
        w_start = (w-nw)//2
        new_image[h_start:h_start+nh, w_start:w_start+nw, :] = image

        if show:
            px = 1/plt.rcParams['figure.dpi']  # pixel in inches
            _, ax = plt.subplots(1, figsize=(new_image.shape[0]*px*2,new_image.shape[1]*px*2))
            _ = ax.imshow(new_image)
        return new_image

    def draw_boxes(self, image, boxes, scores, classes):
        for i, box in enumerate(boxes):
            [top, left, bottom, right] = box
            width, height = right - left, bottom - top
            center_x, center_y = left + width*0.5, top + height*0.5
            score, class_index = scores[i], classes[i]
            label = '{}: {:.4f}'.format(self.class_names[class_index], score) 
            color = tuple([color for color in self.colors[class_index]])
            # drawing with cv2
            cv2.rectangle(image, (left, top), (right, bottom), color, 2)
            cv2.putText(image, label, (left, top), cv2.FONT_HERSHEY_PLAIN, 1, color, 2)

    def evaluate(self, yolo_outputs, image_shape):
        score_thresh = 0.2
        boxes = []
        box_scores = []
        input_shape = np.shape(yolo_outputs[0])[1 : 3]
        input_shape = np.array(input_shape)*32

        for i in range(len(yolo_outputs)):
            _boxes, _box_scores = self.boxes_and_scores(
                yolo_outputs[i], self.anchors[self.anchor_mask[i]], len(self.class_names), 
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
            nms_index_np = self.nms_boxes(class_boxes_np, class_box_scores_np) 
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

    def boxes_and_scores(self, feats, anchors, classes_num, input_shape, image_shape):
        box_xy, box_wh, box_confidence, box_class_probs = self.get_feats(feats, anchors, classes_num, input_shape)
        boxes = self.correct_boxes(box_xy, box_wh, input_shape, image_shape)
        boxes = np.reshape(boxes, [-1, 4])
        box_scores = box_confidence * box_class_probs
        box_scores = np.reshape(box_scores, [-1, classes_num])
        return boxes, box_scores
    
    def get_feats(self, feats, anchors, num_classes, input_shape):
        num_anchors = len(anchors)
        anchors_tensor = np.reshape(np.array(anchors, dtype=np.float32), [1, 1, 1, num_anchors, 2])
        grid_size = np.shape(feats)[1:3]
        nu = num_classes + 5
        predictions = np.reshape(feats, [-1, grid_size[0], grid_size[1], num_anchors, nu])
        grid_y = np.tile(np.reshape(np.arange(grid_size[0]), [-1, 1, 1, 1]), [1, grid_size[1], 1, 1])
        grid_x = np.tile(np.reshape(np.arange(grid_size[1]), [1, -1, 1, 1]), [grid_size[0], 1, 1, 1])
        grid = np.concatenate([grid_x, grid_y], axis = -1)
        grid = np.array(grid, dtype=np.float32)

        box_xy = (1/(1+np.exp(-predictions[..., :2])) + grid) / np.array(grid_size[::-1], dtype=np.float32)
        box_wh = np.exp(predictions[..., 2:4]) * anchors_tensor / np.array(input_shape[::-1], dtype=np.float32)
        box_confidence = 1/(1+np.exp(-predictions[..., 4:5]))
        box_class_probs = 1/(1+np.exp(-predictions[..., 5:]))
        return box_xy, box_wh, box_confidence, box_class_probs
        
    def correct_boxes(self, box_xy, box_wh, input_shape, image_shape):
        box_yx = box_xy[..., ::-1]
        box_hw = box_wh[..., ::-1]
        input_shape = np.array(input_shape, dtype = np.float32)
        image_shape = np.array(image_shape, dtype = np.float32)
        new_shape = np.around(image_shape * np.min(input_shape / image_shape))
        offset = (input_shape - new_shape) / 2. / input_shape
        scale = input_shape / new_shape
        box_yx = (box_yx - offset) * scale
        box_hw *= scale * 10

        box_mins = box_yx - (box_hw / 2.)
        box_maxes = box_yx + (box_hw / 2.)
        boxes = np.concatenate([
            box_mins[..., 0:1],
            box_mins[..., 1:2],
            box_maxes[..., 0:1],
            box_maxes[..., 1:2]
        ], axis = -1)
        boxes *= np.concatenate([image_shape, image_shape], axis = -1)
        return boxes

    def nms_boxes(self, boxes, scores):
        """Suppress non-maximal boxes.

        # Arguments
            boxes: ndarray, boxes of objects.
            scores: ndarray, scores of objects.

        # Returns
            keep: ndarray, index of effective boxes.
        """
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        areas = (x2-x1+1)*(y2-y1+1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w1 = np.maximum(0.0, xx2 - xx1 + 1)
            h1 = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w1 * h1

            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            inds = np.where(ovr <= 0.55)[0]  # threshold
            order = order[inds + 1]

        return keep