#!coding:utf-8
# 巡线算法，默认识别单线，使用hsv颜色空间(180,255,255)
from __future__ import print_function
import cv2
import numpy as np
import os
import yaml
import io
import json


class LineDetector:
    """
    LineDetector 类, 用来检测颜色
    Attributes:
    conf_file: str 配置文件
    min_cnt_area: int 最小轮廓数量,低于该值不会检测
    colors: {name:[(min_h,min_s,min_v),(max_h,max_s,max_v)]} hsv颜色阈值
    size: [width,height] 缩放尺寸
    roi: [x1,y1,x2,y2] 识别区域
    """

    def __init__(self):
        self.conf_file = os.path.dirname(
            os.path.abspath(__file__)) + "/default.yaml"
        stream = io.open(self.conf_file, 'r', encoding='utf-8')
        self.config = yaml.safe_load(stream)
        self.min_cnt_area = self.config[u'最小轮廓']
        self.colors = self.config[u'颜色区间']
        self.size = self.config[u'缩放尺寸']
        self.roi = self.config[u'识别区域']
        # print(self.colors)

    def detect_hsv(self, image, color_hsv):
        """
        detect_hsv 函数, 检测颜色
        Keyword arguments::
        image: image 输入opencv图像
        color_hsv: [(min_h,min_s,min_v),(max_h,max_s,max_v)] hsv颜色阈值
        Returns:
        cnt: 检测轮廓
        image: image 包含轮廓的图像
        """
        # image = self.cv_image
        # x1, x2, y1, y2 = self.roi[0], self.roi[1], self.roi[2], self.roi[3]
        # image = image[y1:y2,x1:x2]
        # image = self.rector.rect(image)
        # image = cv2.resize(image,(self.size[0],self.size[1]))
        # image = cv2.flip(image,-1)
        # cnt_r = detect_cnt(image,[])
        self.cnt = self.detect_cnt(image, color_hsv)
        # if cnt is not None:
        #     cv2.drawContours(image, [self.cnt], -1, (0, 255, 255), thickness=2)
        return self.toLineDetections(self.cnt), image
    def detect_color(self,image,color):
        # print(self.colors,color)
        if image is None:
            return [0,0,0,0,0],None
        color_hsv = self.colors[color]
        return self.detect_hsv(image,color_hsv)
    def detect_cnt(self, image, color_hsv):
        """
        detect_cnt 函数, 检测轮廓
        Keyword arguments::
        image: image 输入opencv图像
                color_hsv: [(min_h,min_s,min_v),(max_h,max_s,max_v)] hsv颜色阈值
                Returns:
                cnt: 检测轮廓
                """
        max_cnt = None
        mask = None
        # ret,thresh1 = cv2.threshold(image,127,255,cv2.THRESH_BINARY)
        # ret,thresh2 = cv2.threshold(image,127,255,cv2.THRESH_BINARY_INV)
        # return findMaxCnt(ret)

        # change to hsv model
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # get mask
        # print(len(color_hsv))
        if len(color_hsv) == 1:
            color = color_hsv[0]
            mask = cv2.inRange(hsv, np.array(
                color['min']), np.array(color['max']))
        elif len(color_hsv) >= 2:
            for color in color_hsv:
                maski = cv2.inRange(hsv, np.array(
                    color['min']), np.array(color['max']))
                if mask is None:
                    mask = maski
                else:
                    mask = cv2.bitwise_or(mask, maski)
        else:
            return max_cnt
        return self.findMaxCnt(mask)

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
        # 腐蚀操作
        # mask = cv2.erode(mask, None, iterations=2)
        # 膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点
        # mask = cv2.dilate(mask, None, iterations=2)
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
            if area > self.min_cnt_area and area > max_area:
                max_cnt = cnt
        return max_cnt
    def toLineDetections(self,cnt):
        if cnt is not None:
            center,wh,angle = cv2.minAreaRect(cnt)
            return [center[0],center[1],wh[0],wh[1],angle]
        return [0,0,0,0,0]
    def crop(self,image,x1,y1,x2,y2):
        if image is None:
            return None
        if y1 < y2 and x1 < x2:
            return image[y1:y2,x1:x2]
        return image

if __name__ == '__main__':
    """
    测试函数，实时检测黄色
    """
    import time
    detector = LineDetector()
    capture = cv2.VideoCapture(0)
    while True:
        # test_image = cv2.imread('./test.png')
        ret, cv_image = capture.read()
        if cv_image is None:
            print('None')
            continue
        start = time.time()
        cnt, image = detector.detect_color(cv_image, u'黄色')
        end = time.time()
        print("detect 1 frame in %.2f ms" %
              ( (end - start)*1000))
        # cnt, image = detector.detect_hsv(cv_image, detector.colors[u'黄线'])
        cv2.imshow("images", image)
        # cv2.imshow("images", np.hstack([image, output]))
        c = cv2.waitKey(2)
        if c == 27:
            break
    capture.release()
