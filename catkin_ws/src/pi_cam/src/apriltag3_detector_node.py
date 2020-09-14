#!/usr/bin/python
#!coding:utf-8
import time
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np
from pi_cam.msg import ApriltagPose
from pi_cam.srv import GetApriltagDetections,GetApriltagDetectionsResponse
# from scipy.spatial.transform import Rotation as R
import os

from camera_utils import load_camera_info_3,bgr_from_jpg,jpg_from_bgr
from apriltag_detector import ApriltagDetector

from pi_cam.srv import GetFrame,GetFrameRequest

class ApriltagDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.bridge = CvBridge()
        self.cv_image = None
        self.detector = ApriltagDetector()
        self.visualization = True

        self.camera_info_msg = load_camera_info_3()
        self.image_msg = None
        self.pub_detections = rospy.Publisher("~image_apriltag", CompressedImage, queue_size=1)

        self.tag_srv = rospy.Service('~detect_apriltag', GetApriltagDetections, self.cbGetApriltagDetections)
        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        self.sub_image = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.cbImg ,  queue_size=1)
        # rospy.loginfo("[%s] wait_for_service : camera_get_frame..." % (self.node_name))
        # rospy.wait_for_service('~camera_get_frame')
        # self.get_frame = rospy.ServiceProxy('~camera_get_frame', GetFrame)
        rospy.loginfo("[%s] Initialized." % (self.node_name))
    def cbImg(self,image_msg):
        self.image_msg = image_msg

    def cbGetApriltagDetections(self,params):
        # res = self.get_frame(GetFrameRequest())
        # res = self.get_frame(GetFrameRequest([320,240]))
        # self.image_msg = res.image
        # print(params)
        image_msg = self.image_msg
        if image_msg == None:
            return GetApriltagDetectionsResponse()
        if hasattr(image_msg,'format'): # CompressedImage
            try:
                cv_image = bgr_from_jpg(image_msg.data)
            except ValueError as e:
                rospy.loginfo('apriltag_detector cannot decode image: %s' % e)
                return
        else: # Image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        rect_image = cv_image
        resized_image = cv2.resize(rect_image,(640,480))
        image_gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(image_gray)
        if self.visualization:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(resized_image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0),2)
                cv2.putText(resized_image, str(tag.tag_id),
                            org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=2,
                            color=(0, 0, 255))
            # imgmsg = self.bridge.cv2_to_imgmsg(rect_image,"bgr8")
            # self.pub_detections.publish(imgmsg)
            resized_image = cv2.resize(resized_image,(480,360))

            self.pubImage(resized_image)

        return self.toApriltagDetections(tags)
    def toApriltagDetections(self,tags):
        msg = GetApriltagDetectionsResponse()
        items = self.detector.toApriltagDetections(tags)
        for item in items:
            detection = ApriltagPose(id=item[0],pose_r=item[1],pose_t=item[2])
            msg.detections.append(detection)
        return msg
    def pubImage(self,image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = jpg_from_bgr(image)
        self.pub_detections.publish(msg)
    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
    rospy.init_node('apriltag_detector_node', anonymous=False)
    node = ApriltagDetectorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
