#!/usr/bin/python3
#!coding:utf-8

import cv2
from pyee import ExecutorEventEmitter
import rospy
from sensor_msgs.msg import CompressedImage
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import FaceDetection
from pi_cam.srv import GetFaceDetections, GetFaceDetectionsResponse
from pi_driver.srv import SetInt32, SetInt32Response
from camera_utils import toImageMsg
from face_recognizer import UltraFaceInference


class UltraFaceInferenceNode(ExecutorEventEmitter):
    def __init__(self):
        super().__init__()
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.visualization = True

        self.msg = GetFaceDetectionsResponse()
        self.boxes = []
        self.getShm()
        self.on('pub_image', self.pubImage)
        # self.pub_detections = rospy.Publisher("~image_ultra_face", Image, queue_size=1)
        self.pub_detections = rospy.Publisher(
            "~image_ultra_face", CompressedImage, queue_size=1)
        rospy.Service('~set_threshold', SetInt32, self.cbSetThreshold)
        rospy.Service('~set_resize', SetInt32, self.srvSetResize)
        self.detector = UltraFaceInference()

        rospy.Service('~detect_face_locations',
                      GetFaceDetections, self.cbDetectFaceLocations)

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

    def cbDetectFaceLocations(self, params):
        # print(params)
        self.rect_image = self.getImage()
        self.boxes, labels, probs = self.detector.detect(self.rect_image)
        # print(self.boxes, labels, probs)
        if self.visualization:
            self.emit('pub_image')
            # faces = self.detector.drawBoxes(rect_image, boxes)
            # msg = toImageMsg(faces)
            # self.pub_detections.publish(msg)
        return self.toFaceDetectionMsg()

    def toFaceDetectionMsg(self):
        msg = self.msg
        for face in self.detector.faceDetections:
            msg.detections.append(FaceDetection(
                "{}".format(round(face[0], 2)), face[1]))
        return msg

    def pubImage(self):
        rect_image = self.rect_image
        boxes = self.boxes
        faces = self.detector.drawBoxes(rect_image, boxes)
        msg = toImageMsg(faces)
        self.pub_detections.publish(msg)

    def cbSetThreshold(self, params):
        if params.value > 0 and params.value < 100:
            self.detector.setThreshold(params.value)
        return SetInt32Response(params.port, params.value)

    def srvSetResize(self, params):
        if params.port <= 480 and params.port >= 10:
            if params.value <= 360 and params.value >= 10:
                self.detector.setResize(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('ultra_face_inference_node', anonymous=False)
    node = UltraFaceInferenceNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
