#!/usr/bin/python3
#!coding:utf-8
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image,CompressedImage
from camera_utils import bgr_from_jpg,jpg_from_bgr
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_driver.srv import GetString,GetStringResponse
from pi_cam.srv import SetRoi,SetRoiResponse

import numpy as np
import pytesseract
from PIL import Image

class TextRecognizerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.roi = [(120,80),(320,280)]
        self.bridge = CvBridge()
        self.visualization = True
        self.image_msg = None
        self.pub_detections = rospy.Publisher("~image_text", CompressedImage, queue_size=1)

        rospy.Service('~detect_text', GetString, self.cbTextDetection)
        rospy.Service('~set_roi', SetRoi, self.cbSetRoi)

        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        self.sub_image = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.cbImg ,  queue_size=1)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbImg(self,image_msg):
        self.image_msg = image_msg
        cv_image = self.getImage(image_msg)
        cv2.rectangle(cv_image,self.roi[0],self.roi[1],(10, 255, 0), 2)
        self.pubImage(cv_image)

    def detect(self,image):
        im = Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))  
        return pytesseract.image_to_string(im)

    def cbSetRoi(self,params):
        if params.x1 < params.x2 and params.x2 <= 480 and params.x1 >=0:
            if params.y1 < params.y2 and params.y2 <= 360 and params.y1 >=0:
                self.roi = [(params.x1,params.y1),(params.x2,params.y2)]
        return SetRoiResponse()

    def cbTextDetection(self,params):
        image = self.getImage(self.image_msg)
        img = image[self.roi[0][1]:self.roi[1][1],self.roi[0][0]:self.roi[1][0]]
        text = self.detect(img)
        return GetStringResponse(text)

    def pubImage(self,image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = jpg_from_bgr(image)
        self.pub_detections.publish(msg)   
    def getImage(self,image_msg):
        if hasattr(image_msg,'format'): # CompressedImage
            try:
                cv_image = bgr_from_jpg(image_msg.data)
            except ValueError as e:
                rospy.loginfo('[%s] cannot decode image: %s' % (self.node_name,e))
                return None
        else: # Image
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg)
            except Exception as e:
                rospy.loginfo('[%s] cannot convert image: %s' % (self.node_name,e))
                return None
        return cv_image
    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
    rospy.init_node('text_recognizer_node', anonymous=False)
    node = TextRecognizerNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()