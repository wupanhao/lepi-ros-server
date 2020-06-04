#!/usr/bin/python3
#!coding:utf-8
import time
import os
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np
from pi_cam.msg import ObjectDetection
from pi_cam.srv import GetObjectDetections,GetObjectDetectionsResponse
from pi_driver.srv import SetInt32,SetInt32Response

from pi_ai import ImageClassifier

from pi_cam.srv import GetFrame,GetFrameRequest

class ImageClassifierNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.IMAGE_W = 224
        self.IMAGE_H = 224
        self.bridge = CvBridge()
        self.visualization = True

        self.pub_image = rospy.Publisher("~image_class", Image, queue_size=1)

        rospy.Service('~class_image', GetObjectDetections, self.cbGetObjectDetections)
        rospy.Service('~set_threshold', SetInt32, self.cbSetThreshold)
        rospy.Service('~set_size', SetInt32, self.srv_set_size)
        self.detector = ImageClassifier()
        try:
            self.detector.load_model(use_TPU=True)
        except Exception as e:
            print(e)
            self.detector.load_model(use_TPU=False)
        self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        rospy.loginfo("[%s] wait_for_service : camera_get_frame..." % (self.node_name))
        rospy.wait_for_service('~camera_get_frame')
        self.get_frame = rospy.ServiceProxy('~camera_get_frame', GetFrame)
        rospy.loginfo("[%s] Initialized." % (self.node_name))
    def getBox(self):
        xmin = (480 - self.IMAGE_W)//2
        ymin = (360 - self.IMAGE_H)//2
        xmax = xmin + self.IMAGE_W
        ymax = ymin + self.IMAGE_H
        return [xmin,ymin,xmax,ymax]
    def cbImg(self,image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg)
        xmin,ymin,xmax,ymax = self.getBox()
        cv2.rectangle(cv_image,(xmin,ymin),(xmax,ymax),(10, 255, 0), 2)
        msg = self.bridge.cv2_to_imgmsg(cv_image,"bgr8")
        self.pub_image.publish(msg)
    def cbGetObjectDetections(self,params):
        res = self.get_frame(GetFrameRequest())
        image_msg = res.image
        image = self.bridge.imgmsg_to_cv2(image_msg)
        xmin,ymin,xmax,ymax = self.getBox()
        image = image[ymin:ymax,xmin:xmax]
        # image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
        detections = self.detector.detect(image)
        # print(detections)
        return self.toObjectDetections(detections)

    def toObjectDetections(self,detections):
        rsp = GetObjectDetectionsResponse()
        for class_,score in detections:
            if score >= self.detector.min_conf_threshold:
                detection = ObjectDetection([0,0,0,0], class_,score*100)
                rsp.detections.append(detection)
        return rsp
    def cbSetThreshold(self,params):
        self.detector.set_threshold(params.value)
        return SetInt32Response(params.port, params.value)

    def srv_set_size(self,params):
        if params.port <= 480 and params.port >= 10:
            self.IMAGE_W = params.port
        if params.value <= 360 and params.value >= 10:
            self.IMAGE_H = params.value
        return SetInt32Response(params.port,params.value)
    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
    rospy.init_node('image_classifier_node', anonymous=False)
    node = ImageClassifierNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
