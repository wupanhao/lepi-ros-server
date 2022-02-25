#!/usr/bin/python
#!coding:utf-8
import rospy
from sensor_msgs.msg import CompressedImage
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

from pi_cam.msg import ApriltagPose
from pi_cam.srv import GetApriltagDetections, GetApriltagDetectionsResponse
# from scipy.spatial.transform import Rotation as R

from camera_utils import load_camera_info_3, toImageMsg, toImage
from apriltag_detector import ApriltagDetector


class ApriltagDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.cv_image = None
        self.detector = ApriltagDetector()
        self.visualization = True

        self.camera_info_msg = load_camera_info_3()
        self.image_msg = None
        self.getShm()
        self.pub_detections = rospy.Publisher(
            "~image_apriltag", CompressedImage, queue_size=1)

        self.tag_srv = rospy.Service(
            '~detect_apriltag', GetApriltagDetections, self.cbGetApriltagDetections)
        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        # self.sub_image = rospy.Subscriber(
        #     "~image_raw/compressed", CompressedImage, self.cbImg,  queue_size=1)

        # start = time.time()
        # for i in range(100):
        #     self.cbGetApriltagDetections(None)
        # print(time.time() - start)
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
        import cv2
        rect_image = self.image_frame.copy()
        return cv2.resize(rect_image, (480, 360))

    def cbImg(self, image_msg):
        self.image_msg = image_msg

    def cbGetApriltagDetections(self, params):
        # res = self.get_frame(GetFrameRequest())
        # res = self.get_frame(GetFrameRequest([320,240]))
        # self.image_msg = res.image
        # print(params)
        # image_msg = self.image_msg
        # if image_msg == None:
        #     return GetApriltagDetectionsResponse()
        # self.rect_image = toImage(image_msg)

        # res = self.get_frame(GetFrameRequest())
        # self.rect_image = toImage(res.image)
        rect_image = self.getImage()
        self.detector.detect(
            rect_image, label_tags=self.visualization)
        if self.visualization:
            self.pubImage(rect_image)

        return self.toApriltagDetections()

    def toApriltagDetections(self):
        msg = GetApriltagDetectionsResponse()
        items = self.detector.detections
        for item in items:
            detection = ApriltagPose(
                id=item[0], pose_r=item[1], pose_t=item[2])
            msg.detections.append(detection)
        return msg

    def pubImage(self, image):
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('apriltag_detector_node', anonymous=False)
    node = ApriltagDetectorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
