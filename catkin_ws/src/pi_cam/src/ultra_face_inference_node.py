#!/usr/bin/python3
#!coding:utf-8

import rospy
from sensor_msgs.msg import CompressedImage
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import FaceDetection
from pi_cam.srv import GetFaceDetections, GetFaceDetectionsResponse
from pi_driver.srv import SetInt32, SetInt32Response
from camera_utils import toImageMsg
from face_recognizer import UltraFaceInference


class UltraFaceInferenceNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.visualization = True

        self.image_msg = None
        self.getShm()
        # self.pub_detections = rospy.Publisher("~image_ultra_face", Image, queue_size=1)
        self.pub_detections = rospy.Publisher(
            "~image_ultra_face", CompressedImage, queue_size=1)
        rospy.Service('~set_threshold', SetInt32, self.cbSetThreshold)
        rospy.Service('~set_resize', SetInt32, self.srvSetResize)
        self.detector = UltraFaceInference()

        rospy.Service('~detect_face_locations',
                      GetFaceDetections, self.cbDetectFaceLocations)
        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg , queue_size=1)
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

    def cbDetectFaceLocations(self, params):
        # print(params)
        rect_image = self.getImage()
        boxes, labels, probs = self.detector.detect(rect_image)
        print(boxes, labels, probs)
        if self.visualization:
            faces = self.detector.drawBoxes(rect_image, boxes)
            msg = toImageMsg(faces)
            self.pub_detections.publish(msg)
        return self.toFaceDetectionMsg()

    def toFaceDetectionMsg(self):
        msg = GetFaceDetectionsResponse()
        for face in self.detector.faceDetections:
            msg.detections.append(FaceDetection(
                "{}".format(round(face[0], 2)), face[1]))
        return msg

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
