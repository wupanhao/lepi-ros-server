#!/usr/bin/python3
#!coding:utf-8
import json
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import toImageMsg, toImage
from pi_driver.srv import GetString, GetStringResponse
from pi_ai import PoseEstimator


class PoseEstimatorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.visualization = True
        self.image_msg = None
        self.detector = PoseEstimator()
        self.getShm()
        self.pub_detections = rospy.Publisher(
            "~image_pose", CompressedImage, queue_size=1)

        rospy.Service('~detect_pose', GetString, self.cbHandDetect)

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
        import cv2
        rect_image = self.image_frame.copy()
        return cv2.resize(rect_image, (480, 360))

    def cbImg(self, image_msg):
        self.image_msg = image_msg

    def cbHandDetect(self, param):
        cv_image = self.getImage()
        result, image = self.detector.detect(cv_image)
        self.pubImage(image)
        return GetStringResponse(json.dumps(result))

    def pubImage(self, image):
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('pose_estimator_node', anonymous=False)
    node = PoseEstimatorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
