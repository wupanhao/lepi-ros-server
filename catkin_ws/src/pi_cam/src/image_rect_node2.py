#!/usr/bin/python

import os.path
import rospkg
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time

from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from camera_utils import load_camera_info_2

class ImageRectNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.count = 0
        self.verbose = True
        # Load parameters
        # TODO cali_file_name is not needed and should be the robot name by default
        self.cali_file_name = self.setupParam("~cali_file_name", "default")

        # Setup publisher
        self.pub_rect = rospy.Publisher("~image_rect", Image, queue_size=1)

        # Get path to calibration yaml file
        self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/" + self.cali_file_name + ".yaml"

        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\nUsing default calibration instead."
                          % (self.node_name, self.cali_file))
            self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Print out and prepare message
        rospy.loginfo("[%s] Using calibration file: %s" % (self.node_name, self.cali_file))
        # Load calibration yaml file
        self.camera_info_msg = load_camera_info_2(self.cali_file)
        rospy.loginfo("[%s] CameraInfo: %s" % (self.node_name, self.camera_info_msg))
        K = np.array(self.camera_info_msg.K).reshape((3,3))
        # D = np.array(self.camera_info_msg.D[:4])
        D = np.array([0.,0.,0.,0.])
        # P = np.array(self.camera_info_msg.P).reshape((3,4))
        DIM = (self.camera_info_msg.width,self.camera_info_msg.height)
        self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

        typemsg = "CompressedImage"
        rospy.logwarn("[%s] ==============%s", self.node_name, typemsg)
        self.sub_img_compressed = rospy.Subscriber("/ubiquityrobot/camera_node/image/compressed", CompressedImage, self.cbCompressedImage, queue_size=1)

    def cbCompressedImage(self, msg):
        self.count+=1
        if self.verbose and self.count > 5:
            time_0= time.time()
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            time_1 = time.time()
            rect_image = cv2.remap(cv_image,self.mapx, self.mapy,cv2.INTER_LINEAR)             
            time_2 = time.time()
            img_msg = self.bridge.cv2_to_imgmsg(rect_image, "bgr8")
            time_3 = time.time()
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = msg.header.frame_id
            self.pub_rect.publish(img_msg)
            time_4 = time.time()
            rospy.loginfo("[%s] time decode %f, rect %f, convert %f publish %f" % (self.node_name,time_1 - time_0,time_2 - time_1, time_3 - time_2 ,time_4 - time_3 ))
            self.count = 0
        else:
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rect_image = cv2.remap(cv_image,self.mapx, self.mapy,cv2.INTER_LINEAR)             
            img_msg = self.bridge.cv2_to_imgmsg(rect_image, "bgr8")
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = msg.header.frame_id
            self.pub_rect.publish(img_msg)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
    rospy.init_node('image_rect_node', anonymous=False)
    node = ImageRectNode()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()