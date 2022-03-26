#!/usr/bin/python3
#!coding:utf-8
from pyee import ExecutorEventEmitter
import cv2
import numpy as np
import json
import time
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import toImageMsg
from pi_driver.srv import GetString, GetStringResponse
from pi_ai import MoveNetPose


class MoveNetPoseNode(ExecutorEventEmitter):
    def __init__(self):
        super().__init__()
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.visualization = True
        self.image_msg = None
        self.detector = MoveNetPose()
        self.results = []
        self.score = 0
        try:
            self.detector.load_model(use_TPU=True)
        except Exception as e:
            print(e)
            self.detector.load_model(use_TPU=False)
        self.getShm()
        self.on('pub_image', self.pubImage)
        self.on('error', self.onError)
        self.pub_detections = rospy.Publisher(
            "~image_pose", CompressedImage, queue_size=1)

        rospy.Service('~detect_pose', GetString, self.cbPoseDetect)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def getShm(self):
        from pi_driver import SharedMemory
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

    def cbPoseDetect(self, param):
        self.cv_image = self.getImage()
        size = self.cv_image.shape
        start = time.time()
        self.results, self.score = self.detector.detect(self.cv_image)
        if self.visualization:
            self.emit('pub_image')
        results = json.loads(json.dumps(self.results))
        print(1000*(time.time()-start))
        for i in range(len(results)):
            x, y, score = results[i]
            results[i][0] = int(y*size[1])
            results[i][1] = int(x*size[0])
            results[i][2] = int(score*100)
        return GetStringResponse(json.dumps(results))

    def pubImage(self):
        image = self.cv_image
        if self.score > 0.5:
            results = self.results
            self.detector.draw_pose(image, results)
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)

    def onError(self, e):
        print(e)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('movenet_pose_node', anonymous=False)
    node = MoveNetPoseNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
