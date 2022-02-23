#!/usr/bin/python3
#!coding:utf-8
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import toImageMsg, toImage
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np


import time

from pi_cam.srv import GetFrame, GetFrameRequest, GetCompressedFrame, GetCompressedFrameRequest


class TestNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        rospy.loginfo("[%s] wait_for_service : camera_get_frame..." % (self.node_name))
        rospy.wait_for_service('/ubiquityrobot/camera_node/camera_get_frame')
        self.get_frame = rospy.ServiceProxy(
            '/ubiquityrobot/camera_node/camera_get_frame', GetFrame)
        self.get_compressed = rospy.ServiceProxy(
            '/ubiquityrobot/camera_node/camera_get_compressed', GetCompressedFrame)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def test_get_frame(self):
        start = time.time()
        for i in range(100):
            res = self.get_frame(GetFrameRequest())
            cv_image = toImage(res.image)
        print(time.time()-start)

    def test_get_compressed(self):
        start = time.time()
        res = self.get_compressed(GetCompressedFrameRequest())
        for i in range(100):
            cv_image = toImage(res.image)
        print(time.time()-start)
        start = time.time()
        for i in range(100):
            res = self.get_compressed(GetCompressedFrameRequest())
            # cv_image = toImage(res.image)
        print(time.time()-start)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=False)
    node = TestNode()
    rospy.on_shutdown(node.onShutdown)
    for i in range(10):
        node.test_get_frame()
        node.test_get_compressed()
        print('-------')
    rospy.spin()
