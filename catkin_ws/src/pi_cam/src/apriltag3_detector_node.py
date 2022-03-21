#!/usr/bin/python
#!coding:utf-8
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from pyee import ExecutorEventEmitter

from pi_cam.msg import ApriltagPose
from pi_cam.srv import GetApriltagDetections, GetApriltagDetectionsResponse
# from scipy.spatial.transform import Rotation as R

from camera_utils import load_camera_info_3, toImageMsg
from apriltag_detector import ApriltagDetector


class ApriltagDetectorNode(ExecutorEventEmitter):
    def __init__(self):
        super().__init__()
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.cv_image = None
        self.detector = ApriltagDetector()
        self.visualization = True

        self.camera_info_msg = load_camera_info_3()
        self.image_msg = None
        self.msg = GetApriltagDetectionsResponse()
        self.getShm()
        self.on('pub_image', self.pubImage)
        self.pub_detections = rospy.Publisher(
            "~image_apriltag", CompressedImage, queue_size=1)
        self.tag_srv = rospy.Service(
            '~detect_apriltag', GetApriltagDetections, self.cbGetApriltagDetections)
        
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def getShm(self):
        from pi_driver import SharedMemory
        import numpy as np
        import time
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

    def cbGetApriltagDetections(self, params):
        self.rect_image = self.getImage()
        self.detector.detect(
            self.rect_image, label_tags=self.visualization)
        if self.visualization:
            self.emit('pub_image')
        return self.toApriltagDetections()

    def toApriltagDetections(self):
        msg = self.msg
        msg.detections = []
        items = self.detector.detections
        for item in items:
            detection = ApriltagPose(
                id=item[0], pose_r=item[1], pose_t=item[2])
            msg.detections.append(detection)
        return msg

    def pubImage(self):
        rect_image = self.rect_image
        self.detector.label_tags(rect_image)
        msg = toImageMsg(rect_image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('apriltag_detector_node', anonymous=False)
    node = ApriltagDetectorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
