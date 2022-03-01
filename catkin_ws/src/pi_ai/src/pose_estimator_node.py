#!/usr/bin/python3
#!coding:utf-8
from pyee import ExecutorEventEmitter
import cv2
import json
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import toImageMsg, toImage
from pi_driver.srv import GetString, GetStringResponse
from pi_ai import PoseEstimator


class PoseEstimatorNode(ExecutorEventEmitter):
    def __init__(self):
        super().__init__()
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.visualization = True
        self.image_msg = None
        self.detector = PoseEstimator()
        self.getShm()
        self.on('pub_image', self.pubImage)
        self.on('error', self.onError)
        self.pub_detections = rospy.Publisher(
            "~image_pose", CompressedImage, queue_size=1)

        rospy.Service('~detect_pose', GetString, self.cbHandDetect)
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

    def cbHandDetect(self, param):
        self.cv_image = self.getImage()
        array, self.results = self.detector.detect(self.cv_image)
        if self.visualization:
            self.emit('pub_image')
        return GetStringResponse(json.dumps(array))

    def pubImage(self):
        image = self.cv_image
        results = self.results
        self.detector.draw_results(image, results)
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)

    def onError(self, e):
        print(e)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('pose_estimator_node', anonymous=False)
    node = PoseEstimatorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
