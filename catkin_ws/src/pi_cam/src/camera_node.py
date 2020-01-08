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

from pi_driver.srv import SetInt32,SetInt32Response,GetStrings,GetStringsResponse
from pi_cam.srv import GetFrame,GetFrameResponse

from usb_camera import UsbCamera
import os
import cv2

class CameraNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.rectify = False
		self.visualization = True
		self.flip_code = 2
		self.rate = 60
		self.size = (320,240)
		self.compress_size = (320,240)
		self.bridge = CvBridge()
		self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
		self.camera = UsbCamera(rate=self.rate,callback = self.cbImg)
		self.cv_image = None
		self.lost_count = 0

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
		rospy.Service('~get_image_topics', GetStrings, self.srvGetImageTopics)
		# self.pid_set_enable_srv = rospy.Service('~pid_set_enable', SetPid, self.cbPidSetEnable)
		# self.pub_image_detection = rospy.Publisher("~image_detections", Image, queue_size=1)

		rospy.Timer(rospy.Duration.from_sec(1.0/15), self.cbPublish)
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
		self.flip_code = params.value
		return SetInt32Response(0,self.flip_code)
	def srvCameraSetRectify(self,params):
		if params.value == 1:
			self.rectify = True
			return SetInt32Response(1,params.value)
		elif params.value == 0:
			self.rectify = False
			return SetInt32Response(1,params.value)
		return SetInt32Response()
	def srvCameraGetFrame(self,params):
		if self.image_msg is None:
			return GetFrameResponse()
		return GetFrameResponse(self.image_msg)

	def cbImg(self,cv_image):
		self.cv_image = cv_image

	def cbPublish(self,channel):
		if self.camera.active == False or self.cv_image is None:
			return
		cv_image = self.cv_image
		if self.rectify:
			cv_image = self.rector.rect(cv_image)
		cv_image = cv2.resize(cv_image,(480,360))
		if abs(self.flip_code) <= 1:
			cv_image = cv2.flip(cv_image,self.flip_code)			
		image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		image_msg.header.stamp = rospy.Time.now()
		image_msg.header.frame_id = self.frame_id
		self.image_msg = image_msg
		self.pub_raw.publish(image_msg)		

	def srvGetImageTopics(self,params):
		topics = rospy.get_published_topics()
		res = GetStringsResponse()
		for topic in topics:
			if topic[1] == 'sensor_msgs/Image':
				res.data.append(topic[0])
		return res

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
