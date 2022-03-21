#!/usr/bin/python3
#!coding:utf-8
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import toImageMsg, toImage
from pi_driver.srv import GetString, GetStringResponse
from pi_cam.srv import SetRoi, SetRoiResponse
from paddleocr import PaddleOCR

class TextRecognizerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.roi = [(120, 120), (360, 240)]
        self.visualization = True
        self.image_msg = None
        self.getShm()
        # 支持 `ch`, `en`, `fr`, `german`, `korean`, `japan`
        # --use_angle_cls true设置使用方向分类器识别180度旋转文字
        self.ocr = PaddleOCR(use_angle_cls=False, lang='ch')
        # self.pub_detections = rospy.Publisher("~image_text", CompressedImage, queue_size=1)

        rospy.Service('~detect_text', GetString, self.cbTextDetection)
        rospy.Service('~set_roi', SetRoi, self.cbSetRoi)

        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        # self.sub_image = rospy.Subscriber(
        #     "~image_raw/compressed", CompressedImage, self.cbImg,  queue_size=1)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def getShm(self):
        from pi_driver import SharedMemory
        import time
        import numpy as np
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

    def cbImg(self, image_msg):
        self.image_msg = image_msg
        return

    def detect(self, image, lang=None):
        result = self.ocr.ocr(image, cls=False)
        print(result)
        text = ''
        for line in result:
            text = text+line[1][0]+' '
        return text
        
    def cbSetRoi(self, params):
        if params.x1 < params.x2 and params.x2 <= 480 and params.x1 >= 0:
            if params.y1 < params.y2 and params.y2 <= 360 and params.y1 >= 0:
                self.roi = [(params.x1, params.y1), (params.x2, params.y2)]
                return SetRoiResponse('设置成功')
        return SetRoiResponse('参数无效,设置失败')

    def cbTextDetection(self, params):
        image = self.getImage()
        img = image[self.roi[0][1]:self.roi[1]
                    [1], self.roi[0][0]:self.roi[1][0]]
        text = ''
        try:
            text = self.detect(img, params.data)
        except Exception as e:
            print(e)
        return GetStringResponse(text)

    def pubImage(self, image):
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('text_recognizer_node', anonymous=False)
    node = TextRecognizerNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
