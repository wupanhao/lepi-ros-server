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
from pi_cam.srv import GetFrame,GetFrameResponse
from usb_camera import UsbCamera
import os
import cv2

class CameraNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.rectify = True
		self.flip_code = -1
		self.rate = 60
		self.size = (640,480)
		self.compress_size = (320,240)
		self.bridge = CvBridge()
		self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
		self.camera = UsbCamera(rate=self.rate,callback = self.cbImg)
		self.cv_image = None

		# self.r = rospy.Rate(self.rate)
		self.rector = ImageRector()

		self.cali_file = os.path.dirname(os.path.abspath(__file__)) + "/default.yaml"
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		self.camera_info_msg_rect = load_camera_info_2(self.cali_file)
		self.image_msg = None # Image()
		self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)
		# self.pub_rect = rospy.Publisher("~image_rect", Image, queue_size=1)
		self.pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)

		# self.pub_compressed = rospy.Publisher("~image_raw/compressed", CompressedImage, queue_size=1)
		# self.pub_camera_info_rect = rospy.Publisher("~rect/camera_info", CameraInfo, queue_size=1)
		rospy.Service('~camera_set_enable', SetInt32, self.srvCameraSetEnable)
		rospy.Service('~camera_set_flip', SetInt32, self.srvCameraSetFlip)
		rospy.Service('~camera_set_rectify', SetInt32, self.srvCameraSetRectify)
		rospy.Service('~camera_get_frame', GetFrame, self.srvCameraGetFrame)
		# rospy.Timer(rospy.Duration.from_sec(1.0/15), self.publishRaw)
		rospy.loginfo("[%s] Initialized......" % (self.node_name))
	def srvCameraSetEnable(self,params):
		if params.value == 1 and self.camera.active == False:
			ret = self.camera.open_camera(params.port)
			return SetInt32Response(1,ret)
		elif params.value == 0:
			self.camera.active = False
			return SetInt32Response(1,0)
		return SetInt32Response()
	def srvCameraSetFlip(self,params):
		if -1 <= params.value and params.value<=1:
			self.flip_code = params.value
			return SetInt32Response(0,self.flip_code)
		else:
			return SetInt32Response(1,1)
	def srvCameraSetRectify(self,params):
		if params.value == 1:
			self.rectify = True
			return SetInt32Response(1,params.value)
		elif params.value == 0:
			self.rectify = False
			return SetInt32Response(1,params.value)
		return SetInt32Response()
	def srvCameraGetFrame(self,params):
		if self.cv_image is None:
			return GetFrameResponse()
		cv_image = self.cv_image
		if self.rectify:
			cv_image = self.rector.rect(cv_image)
		if self.flip_code is not None:
			cv_image = cv2.flip(cv_image,self.flip_code)		
		if len(params.size)==2 and params.size[0]>0 and params.size[1]>0:
			size = (params.size[0],params.size[1])
			print(size)
			cv_image = cv2.resize(cv_image,size)
		image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		image_msg.header.stamp = rospy.Time.now()
		image_msg.header.frame_id = self.frame_id
		return GetFrameResponse(image_msg)
	def cbImg(self,cv_image):
		self.cv_image = cv_image
		self.publishRaw()
	def publishRaw(self,event=None):
		if self.cv_image is None:
			return
		cv_image = self.cv_image
		if self.rectify:
			cv_image = self.rector.rect(cv_image)
		if self.flip_code is not None:
			cv_image = cv2.flip(cv_image,self.flip_code)
		# Publish raw image
		cv_image = cv2.resize(cv_image,self.compress_size)
		image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		image_msg.header.stamp = rospy.Time.now()
		image_msg.header.frame_id = self.frame_id
		self.pub_raw.publish(image_msg)

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
