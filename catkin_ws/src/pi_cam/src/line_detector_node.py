#!/usr/bin/python
#!coding:utf-8
from pyee import ExecutorEventEmitter
import rospy
import time
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from line_detector import LineDetector
from pi_cam.srv import GetLineDetection,GetLineDetectionResponse
from pi_cam.srv import SetColorThreshold,SetColorThresholdResponse,GetColorThreshold,GetColorThresholdResponse
from pi_cam.msg import LineDetection
from pi_driver.srv import GetStrings,GetStringsResponse
from camera_utils import toImageMsg
from pi_driver import SharedMemory
# from std_msgs.msg import UInt8,Int32
class LineDetectorNode(ExecutorEventEmitter):
    def __init__(self):
        super().__init__()
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.cv_image = None
        self.detector = LineDetector()
        # 优化结果可视化更节省时间
        self.visualization = True
        self.line_msg = GetLineDetectionResponse()
        self.getShm()
        self.on('pub_image',self.pubImage)
        self.pub_detections = rospy.Publisher("~image_color", CompressedImage, queue_size=1)
        self.detect_line_srv = rospy.Service('~detect_line', GetLineDetection, self.cbGetLineDetection)
        self.set_color_srv = rospy.Service('~set_color_threshold', SetColorThreshold, self.cbSetColorThreshold)
        self.get_color_srv = rospy.Service('~get_color_threshold', GetColorThreshold, self.cbGetColorThreshold)
        self.set_color_list_srv = rospy.Service('~get_color_list', GetStrings, self.cbGetColorList)

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def getShm(self):
        while True:
            try:
                self.shm = SharedMemory('cv_image')
                self.image_frame = np.ndarray(
                    (480, 640, 3), dtype=np.uint8, buffer=self.shm.buf)
                break
            except:
                print(self.node_name, 'wait for SharedMemory cv_image')
                time.sleep(1)

    def getImage(self):
        rect_image = self.image_frame.copy()
        return cv2.resize(rect_image, (480, 360))

    def detectLine(self,params):
        # start = time.time()
        # print(params)
        if not isinstance(params.color, str):
            color = params.color.decode('utf-8')
        else:
            color = params.color
        if color not in self.detector.colors:
            return GetLineDetectionResponse()
        self.cv_image = self.getImage()
        if params.y1 < params.y2 and params.x1 < params.x2:
            self.params = params
            rect_image = self.cv_image[params.y1:params.y2,
                                       params.x1:params.x2]
        detection,image = self.detector.detect_hsv(rect_image,self.detector.colors[color])
        if self.visualization:
            self.emit('pub_image')
        # end = time.time()
        # print('time cost in detect line: %.2f ms' %  ((end - start)*1000))
        return GetLineDetectionResponse(detection[0:2],detection[2:4],detection[4])

    def cbGetLineDetection(self,params):
        try:
            line_msg = self.detectLine(params)
            return line_msg
        except Exception as e:
            print(self.node_name)
            print(e)
            return GetLineDetectionResponse()
        # return self.line_msg
    def cbSetColorThreshold(self,params):
        print(params)
        if not isinstance(params.color, str):
            color = params.color.decode('utf-8')
        else:
            color = params.color
        try:
            self.detector.colors[color] = [{"min": [params.low_h, params.low_s, params.low_v],
                                                    "max":[params.high_h,params.high_s,params.high_v]}]
            return SetColorThresholdResponse("设置成功")
        except Exception as e:
            print(self.node_name)
            print(e)
            return SetColorThresholdResponse("设置失败,请检查参数")
    def cbGetColorThreshold(self,params):
        if not isinstance(params.color, str):
            color = params.color.decode('utf-8')
        else:
            color = params.color
        try:
            threshold = self.detector.colors[color]
            return GetColorThresholdResponse(str(threshold))
        except Exception as e:
            print(self.node_name)
            print(e)
            return GetColorThresholdResponse()
    def cbGetColorList(self,params):
        return GetStringsResponse(self.detector.colors.keys())

    def pubImage(self):
        params = self.params
        cv_image = self.cv_image
        cnt = self.detector.cnt
        if cnt is not None:
            image = cv_image[params.y1:params.y2,
                                 params.x1:params.x2]
            cv2.drawContours(image, [cnt], -1,
                                 (0, 255, 255), thickness=2)
            cv_image[params.y1:params.y2, params.x1:params.x2] = image
        cv2.rectangle(cv_image, (params.x1, params.y1), (params.x2, params.y2), (0, 0, 0), 1)
        self.pub_detections.publish(toImageMsg(cv_image))

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
    rospy.init_node('line_detector_node', anonymous=False)
    node = LineDetectorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
