#!/usr/bin/python3
#!coding:utf-8
from pyee import ExecutorEventEmitter
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import toImageMsg
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import ObjectDetection
from pi_cam.srv import GetObjectDetections, GetObjectDetectionsResponse
from pi_driver.srv import SetInt32, SetInt32Response

from pi_ai import ObjectDetector

class ObjectDetectorNode(ExecutorEventEmitter):
    def __init__(self):
        super().__init__()
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.visualization = True
        self.getShm()
        self.on('pub_image',self.pubImage)
        self.pub_detections = rospy.Publisher(
            "~image_object", CompressedImage, queue_size=1)
        rospy.Service('~detect_object', GetObjectDetections,
                      self.cbGetObjectDetections)
        rospy.Service('~set_threshold', SetInt32, self.cbSetThreshold)
        self.detector = ObjectDetector()
        try:
            self.detector.load_model(use_TPU=True)
        except Exception as e:
            print(e)
            self.detector.load_model(use_TPU=False)
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

    def cbGetObjectDetections(self, params):
        self.image = self.getImage()
        self.boxes, self.classes, self.scores = self.detector.detect(
            self.image)
        if self.visualization:
            self.emit('pub_image')
        return self.toObjectDetections(self.boxes, self.classes, self.scores)

    def toObjectDetections(self, boxes, classes, scores):
        rsp = GetObjectDetectionsResponse()
        for i, score in enumerate(scores):
            if score >= self.detector.min_conf_threshold:
                detection = ObjectDetection(self.detector.getRealBox(
                    boxes[i]), self.detector.labels[int(classes[i])], score*100)
                rsp.detections.append(detection)
        return rsp

    def cbSetThreshold(self, params):
        self.detector.set_threshold(params.value)
        return SetInt32Response(params.port, params.value)

    def pubImage(self):
        image = self.image
        boxes, classes, scores = (self.boxes, self.classes, self.scores)
        image = self.detector.draw_labels(image, boxes, classes, scores)
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)


    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('object_detector_node', anonymous=False)
    node = ObjectDetectorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
