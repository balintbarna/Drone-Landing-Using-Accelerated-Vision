import numpy as np

import sys
from os.path import join, pardir, dirname
sys.path.append(join(dirname(__file__), pardir, "dai_jupyter"))

from dai_jupyter.nonb.cam import Camera
cam = Camera()

from pynq_dpu import DpuOverlay
overlay = DpuOverlay("dpu.bit")
overlay.load_model(join(dirname(__file__), pardir, "yolo/dk_yolov3_voc_416_416.xmodel"))

dpu = overlay.runner

inputTensors = dpu.get_input_tensors()
print("Input Tensors: " + str(inputTensors))
outputTensors = dpu.get_output_tensors()
print("Output Tensors: " + str(outputTensors))

inShapes = []
for tensor in inputTensors:
    inShapes.append(tuple(tensor.dims))
print("Input Shapes: " + str(inShapes))
outShapes = []
for tensor in outputTensors:
    outShapes.append(tuple(tensor.dims))
print("Output Shapes: " + str(outShapes))

im_res = (inShapes[0][1], inShapes[0][2])
print("Required input image resolution: " + str(im_res))

output_data = []
for shape in outShapes:
    output_data.append(np.empty(shape, dtype=np.float32, order="C"))
input_data = []
for shape in inShapes:
    input_data.append(np.empty(shape, dtype=np.float32, order="C"))
image = input_data[0]

from dai_jupyter.yolo.evaluator import Evaluator
evaluator = Evaluator()

def detect(img, label="boat", display=False):
    preprocessed = np.array(evaluator.pre_process(img, im_res), dtype=np.float32)
    image[0,...] = preprocessed.reshape(image.shape[1:])
    job_id = dpu.execute_async(input_data, output_data)
    dpu.wait(job_id)
    image_size = img.shape[:2]
    boxes, scores, classes = evaluator.evaluate(output_data, image_size)
    if display:
        evaluator.draw_boxes(img, boxes, scores, classes)
        px = 1/plt.rcParams['figure.dpi']  # pixel in inches
        _, ax = plt.subplots(1, figsize=(image_size[0]*px*2,image_size[1]*px*2))
        _ = ax.imshow(img)
    for i in range(len(classes)):
        if evaluator.class_names[classes[i]] == label:
            return boxes[i]
    return None

from dai_jupyter.nonb.box_publisher import BoxPublisher
pub = BoxPublisher()

def publish_box(box, debug = False):
    if box is not None:
        # box is top, left, bottom, right
        h, w, _ = img.shape
        [top, left, bottom, right] = box
        nbox = [
            top/h,
            left/w,
            bottom/h,
            right/w
        ]
        pub.publish(nbox, debug)

import time
fps = 0
ratio = 0.9

while pub.active:
    startTime = time.time()
    img = cam.get_rgb()
    box = detect(img, display=False)
    publish_box(box, debug=False)
    executionTime = (time.time() - startTime)
    fps = fps*ratio + (1/executionTime)*(1-ratio)

print("Average FPS: " + str(fps))

cam.close()
pub.close()
