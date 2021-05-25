#!coding: utf-8
import cv2
import numpy as np
import json


class Procedure:
    def __init__(self, func, args):
        self.func = func
        self.args = args

    def __repr__(self):
        return [self.func, self.args]


class ImageProcessor:
    def __init__(self):
        self.steps = [
        ]
        self.funcNameMap = {
            "cvtColor": self.cvtColor,
            "threshold": self.threshold,
            "adaptiveThresholdGaussian": self.adaptiveThresholdGaussian,
            "medianBlur": self.medianBlur,
            "GaussianBlur": self.GaussianBlur,
            "blur": self.blur,
            "erode": self.erode,
            "dilate": self.dilate,
            "Canny": self.Canny,
        }

    def cvtColor(self, image, code):
        # code=cv2.COLOR_BGR2GRAY
        # code = kwargs['code']
        return cv2.cvtColor(image, code)

    def threshold(self, image, thresh=0, code=cv2.THRESH_BINARY):
        maxVal = 255
        img = image
        if len(image.shape) == 3:
            img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if thresh == 0:
            code = code + cv2.THRESH_OTSU
        ret, mask = cv2.threshold(img, thresh, maxVal, code)
        return mask

    def adaptiveThresholdGaussian(self, image, size=11, sigma=2):
        # 自适应高斯阈值
        img = image
        if len(image.shape) == 3:
            img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, size, sigma)

    def medianBlur(self, img, ksize=3):
        # 中值滤波
        return cv2.medianBlur(img, ksize=ksize)

    def GaussianBlur(self, img, size=(5, 5), sigmaX=0, sigmaY=0):
        # 高斯滤波
        return cv2.GaussianBlur(img, tuple(size), sigmaX, sigmaY)

    def blur(self, img, size=(5, 5)):
        # 均值滤波
        return cv2.blur(img, tuple(size))

    def erode(self, img, size=(5, 5), iterations=1):
        kernel = np.ones(tuple(size), np.uint8)
        return cv2.erode(img, kernel, iterations)

    def dilate(self, img, size=(5, 5), iterations=1):
        kernel = np.ones(tuple(size), np.uint8)
        return cv2.dilate(img, kernel, iterations)

    def Canny(self, img, min, max):
        return cv2.Canny(img, min, max)

    def findMaxCnt(self, mask):
        """
        findMaxCnt 函数, 查找最大轮廓
        Keyword arguments::
        mask: image 掩码图像
        Returns:
        max_cnt: 检测到的最大的轮廓
        """
        if mask is None:
            return None
        # _,contours,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        res = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(res) == 2:
            contours, hierarchy = res
        elif len(res) == 3:
            _, contours, hierarchy = res
        max_cnt = None
        max_area = 0
        cnts = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 200 and area > max_area:
                max_cnt = cnt
        return max_cnt

    def toLineDetections(self, cnt):
        if cnt is not None:
            center, wh, angle = cv2.minAreaRect(cnt)
            return [center[0], center[1], wh[0], wh[1], angle]
        return [0, 0, 0, 0, 0]

    def process(self, image):
        for step in self.steps:
            print(step.func, step.args)
            func = self.funcNameMap[step.func]
            img = func(image, **step.args)
            image = img
        return image

    def addProc(self, name, args):
        if name in self.funcNameMap:
            self.steps.append(Procedure(name, json.loads(args)))
        elif name == "clearProcess":
            self.steps = []

    def testArgs(self, **kwargs):
        print(kwargs)


def nothing(x):
    pass


def cannyTest():
    dev = cameraList()
    cap = cv2.VideoCapture(dev[0])
    cv2.namedWindow('res')
    cv2.createTrackbar('min', 'res', 0, 255, nothing)
    cv2.createTrackbar('max', 'res', 0, 255, nothing)
    cv2.setTrackbarPos('min', 'res', 80)
    cv2.setTrackbarPos('max', 'res', 200)
    while True:
        _, img = cap.read()
        if cv2.waitKey(1) & 0xFF == 27:
            break
        maxVal = cv2.getTrackbarPos('max', 'res')
        minVal = cv2.getTrackbarPos('min', 'res')
        canny = cv2.Canny(img, minVal, maxVal)
        cv2.imshow('res', canny)
    cv2.destroyAllWindows()


def test():
    import time
    processer = ImageProcessor()
    proces = [Procedure(processer.cvtColor, {"code": cv2.COLOR_BGR2GRAY}),
              #   Procedure(processer.threshold, {
              #             "thresh": 0, "maxVal": 255, "code": cv2.THRESH_BINARY}),
              Procedure(processer.Canny, {"min": 30, "max": 120}),
              ]
    processer.steps.extend(proces)

    dev = cameraList()
    cap = cv2.VideoCapture(dev[0])
    while True:
        # Grab a single frame of video
        ret, frame = cap.read()
        start = time.time()
        # processed = processer.adaptiveThresholdGaussian(frame, 11, 2)
        # processed = processer.medianBlur(frame, 3)
        # processed = processer.GaussianBlur(frame, (11, 11))
        # processed = processer.blur(frame, (5, 5))
        # processed = processer.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # processed = processer.threshold(
        #     frame, code=cv2.THRESH_BINARY)  # cv2.THRESH_BINARY_INV
        # processed = processer.erode(processed)
        # processed = processer.dilate(processed)
        processed = processer.process(frame)
        end = time.time()
        print("process %d frame in %.2f ms" %
              (1, (end - start)*1000))
        cv2.imshow("images", processed)
        c = cv2.waitKey(4)
        if c == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


def testDetectLine():
    aera = [0, 480, 300, 360]
    x1, x2, y1, y2 = aera
    processer = ImageProcessor()
    dev = cameraList()
    cap = cv2.VideoCapture(dev[0])
    while True:
        # Grab a single frame of video
        ret, frame = cap.read()
        frame = frame[y1:y2, x1:x2]
        image = frame
        start = time.time()
        processed = processer.threshold(
            frame, thresh=200, code=cv2.THRESH_BINARY_INV)  # cv2.THRESH_BINARY_INV
        processed = processer.erode(processed)
        processed = processer.dilate(processed)
        cnt = processer.findMaxCnt(processed)
        cv2.drawContours(image, [cnt], -1, (0, 255, 255), thickness=2)
        end = time.time()
        print("process %d frame in %.2f ms" %
              (1, (end - start)*1000))
        cv2.imshow("images", image)
        cv2.imshow("processed", processed)
        c = cv2.waitKey(4)
        if c == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    import time
    from camera_utils import cameraList
    # cannyTest()
    test()
    # testDetectLine()
    # processer = ImageProcessor()
    # param = {'x': 1, 'y': 2, 'z': 3}
    # processer.testArgs(**param)
