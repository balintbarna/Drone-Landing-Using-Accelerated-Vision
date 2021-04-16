import cv2
from matplotlib import pyplot as plt

class Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def get_img(self):
        ret, img = self.cap.read()
        return img

    def get_rgb(self):
        img = self.get_img()
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return rgb
    
    def get_bgr(self):
        return self.get_img()

    def close(self):
        self.cap.release()

if __name__ == "__main__":
    cam = Camera()
    for i in range(0, 5):
        rgb = cam.get_rgb()
        plt.imshow(rgb)
        plt.show()
    cam.close()