import numpy as np

def calculate_softmax(data):
    result = np.exp(data)
    return result

def predict_label(softmax, path):
    with open(path, "r") as f:
        lines = f.readlines()
    return lines[np.argmax(softmax)-1]