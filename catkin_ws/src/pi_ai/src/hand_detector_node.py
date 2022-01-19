#!/usr/bin/python3
#!coding:utf-8
import json
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import toImageMsg, toImage
from pi_driver.srv import GetString, GetStringResponse
from pi_ai import HandDetector


class HandDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.visualization = True
        self.image_msg = None
        self.detector = HandDetector()
        self.pub_detections = rospy.Publisher(
            "~image_hand", CompressedImage, queue_size=1)

        rospy.Service('~detect_hand', GetString, self.cbHandDetect)

        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        self.sub_image = rospy.Subscriber(
            "~image_raw/compressed", CompressedImage, self.cbImg,  queue_size=1)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbImg(self, image_msg):
        self.image_msg = image_msg

    def cbHandDetect(self, param):
        cv_image = toImage(self.image_msg)
        result, image = self.detector.detect(cv_image)
        self.pubImage(image)
        return GetStringResponse(json.dumps(result))

    def pubImage(self, image):
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('hand_detector_node', anonymous=False)
    node = HandDetectorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
