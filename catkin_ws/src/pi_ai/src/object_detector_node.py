#!/usr/bin/python3
#!coding:utf-8
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import bgr_from_jpg, jpg_from_bgr
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import ObjectDetection
from pi_cam.srv import GetObjectDetections, GetObjectDetectionsResponse
from pi_driver.srv import SetInt32, SetInt32Response

from pi_ai import ObjectDetector

class ObjectDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.visualization = True
        self.image_msg = None
        self.getShm()
        self.pub_detections = rospy.Publisher(
            "~image_object", CompressedImage, queue_size=1)
        # self.sub_image = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.cbImg ,  queue_size=1)

        rospy.Service('~detect_object', GetObjectDetections,
                      self.cbGetObjectDetections)
        rospy.Service('~set_threshold', SetInt32, self.cbSetThreshold)
        self.detector = ObjectDetector()
        try:
            self.detector.load_model(use_TPU=True)
        except Exception as e:
            print(e)
            self.detector.load_model(use_TPU=False)
        # rospy.loginfo("[%s] wait_for_service : camera_get_frame..." % (self.node_name))
        # rospy.wait_for_service('~camera_get_frame')
        # self.get_frame = rospy.ServiceProxy('~camera_get_frame', GetFrame)
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

    def cbGetObjectDetections(self, params):
        image = self.getImage()
        boxes, classes, scores = self.detector.detect(image)
        if self.visualization:
            image = self.detector.draw_labels(image, boxes, classes, scores)
        self.pubImage(image)
        return self.toObjectDetections(boxes, classes, scores)

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

    def cbImg(self, image_msg):
        self.image_msg = image_msg

    def pubImage(self, image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = jpg_from_bgr(image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('object_detector_node', anonymous=False)
    node = ObjectDetectorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
