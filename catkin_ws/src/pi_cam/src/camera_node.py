#!/usr/bin/python
#!coding:utf-8
import time
# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

from camera_utils import load_camera_info_2
from image_rector import ImageRector

from pi_driver.srv import SetInt32,SetInt32Response
from usb_camera import UsbCamera
import os

class CameraNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.rectify = False
		self.rate = 20
		self.bridge = CvBridge()
		self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
		self.camera = UsbCamera(rate=self.rate,callback = self.PublishRaw)
		self.cv_image = None

		# self.r = rospy.Rate(self.rate)
		self.rector = ImageRector()

		self.cali_file = os.path.dirname(os.path.abspath(__file__)) + "/default.yaml"
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		self.camera_info_msg_rect = load_camera_info_2(self.cali_file)
		self.image_msg = None # Image()
		self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)
		self.pub_rect = rospy.Publisher("~image_rect", Image, queue_size=1)
		self.pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)

		# self.pub_compressed = rospy.Publisher("~image_raw/compressed", CompressedImage, queue_size=1)
		# self.pub_camera_info_rect = rospy.Publisher("~rect/camera_info", CameraInfo, queue_size=1)

		rospy.Service('~camera_set_enable', SetInt32, self.srvCameraSetEnable)

		rospy.loginfo("[%s] Initialized......" % (self.node_name))
	def srvCameraSetEnable(self,params):
		if params.value == 1 and self.camera.active == False:
			ret = self.camera.open_camera(params.port)
			return SetInt32Response(0,ret)
		elif params.value == 0:
			self.camera.active = False
			return SetInt32Response(0,0)
		else:
			return SetInt32Response(1,1)
	def PublishRaw(self,cv_image):
		# Publish raw image
		self.cv_image = cv_image
		img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		# time_2 = time.time()
		img_msg.header.stamp = img_msg.header.stamp
		img_msg.header.frame_id = img_msg.header.frame_id
		self.pub_raw.publish(img_msg)
		self.image_msg = img_msg
		if self.rectify:
			rect_image = self.rector.rect(cv_image)
			img_msg_rect = self.bridge.cv2_to_imgmsg(rect_image, "bgr8")
			img_msg_rect.header = img_msg.header
			self.pub_rect.publish(img_msg_rect)

	def onShutdown(self):
		rospy.loginfo("[%s] Closing camera." % (self.node_name))
		self.is_shutdown = True
		rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
	rospy.init_node('camera_node', anonymous=False)
	camera_node = CameraNode()
	rospy.on_shutdown(camera_node.onShutdown)
	#thread.start_new_thread(camera_node.startCaptureRawCV, ())
	# thread.start_new_thread(camera_node.startCaptureCompressed, ())
	rospy.spin()
