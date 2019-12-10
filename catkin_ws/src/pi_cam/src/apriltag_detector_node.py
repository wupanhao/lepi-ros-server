#!/usr/bin/env python
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
# from duckietown_msgs.msg import BoolStamped

import yaml
import time
import sys
from scipy.spatial.transform import Rotation as R

from apriltag_detector import ApriltagDetector

test_images_path = 'test'
visualization = True

class ApriltagDetectorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.active = True
        self.bridge = CvBridge()

        self.detector = ApriltagDetector()

        self.publish_freq = self.setupParam("~publish_freq", 10.0)
        self.pub_raw = rospy.Publisher("~tag_detections",Image,queue_size=1)
        self.last_stamp = rospy.Time.now()
        self.sub_compressed_img = rospy.Subscriber("/image_rect_node/image_rect",Image,self.cbImg,queue_size=1)
        # self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self,switch_msg):
        self.active = switch_msg.data

    def cbImg(self,image_message):
        cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        tags = self.detector.detect(cv_image)
        if visualization:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(cv_image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
                cv2.putText(cv_image, str(tag.tag_id),
                            org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.8,
                            color=(0, 0, 255))
            imgmsg = self.bridge.cv2_to_imgmsg(cv_image,"bgr8")
            self.pub_raw.publish(imgmsg)
        if len(tags) > 0:
            tag = tags[0]
            base = np.eye(3)
            r = R.from_dcm(np.array(tag.pose_R))
            offset = np.array(tag.pose_t)*100
            euler = r.as_euler('xyz', degrees=True)
            print('tag id %d wx:%f wy:%f wz:%f x:%f y:%f z:%f'% ( tag.tag_id , euler[0], euler[1],euler[2] ,offset[0],offset[1],offset[2] ) )
        else:
            print('no tag detected')
if __name__ == '__main__':
    rospy.init_node('apriltag_detector_node',anonymous=False)
    node = ApriltagDetectorNode()
    rospy.spin()
