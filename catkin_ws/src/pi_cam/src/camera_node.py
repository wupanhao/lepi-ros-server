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
from pi_cam.srv import GetFrame,GetFrameResponse,SetPid,SetPidResponse

from usb_camera import UsbCamera
import os
import cv2
from pi_driver import CarDriver

from line_detector import LineDetector

class CameraNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.rectify = True
		self.visualization = True
		self.flip_code = -1
		self.rate = 60
		self.size = (320,240)
		self.compress_size = (320,240)
		self.bridge = CvBridge()
		self.detector = LineDetector()
		self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
		self.camera = UsbCamera(rate=self.rate,callback = self.cbImg)
		self.cv_image = None
		self.pid_enabled = False
		self.base_speed = 50
		self.car = CarDriver()
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

		# rospy.Timer(rospy.Duration.from_sec(1.0/15), self.pidControl)
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
		if len(params.size)==2 and params.size[0]>0 and params.size[1]>0:
			size = (params.size[0],params.size[1])
			print(size)
			cv_image = cv2.resize(cv_image,size)
		image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		image_msg.header.stamp = rospy.Time.now()
		image_msg.header.frame_id = self.frame_id
		return GetFrameResponse(image_msg)
	def cbPid(self,cnt):
		if cnt is not None:
			center,wh,angle = cv2.minAreaRect(cnt)
		else:
			self.lost_count = self.lost_count + 1
			print('lost count %d' % (self.lost_count))
			if self.lost_count > 25:
				self.pid_enabled = False
				print('yellow line lost,stop')
			self.car.setWheelsSpeed(0,0)
			return
		self.lost_count = 0
		offset = self.offset - center[0]*2
		bias = offset/640.0*self.factor
		speed = self.base_speed/100.0
		left = speed*(1-bias)
		right = speed*(1+bias)
		self.car.setWheelsSpeed(left,right)
	def detectLine(self,cv_image):
		#cv_image = self.cv_image
		start = time.time()
		if cv_image.shape[0]!=self.size[1] or cv_image.shape[1]!=self.size[0]:
			cv_image = cv2.resize(cv_image,self.size)
		rect_image = cv_image[120:140,:]
		cnt,image = self.detector.detect_color(rect_image,self.detector.colors[u'黄线'])
		self.cbPid(cnt)
		if self.visualization:
			cv_image[120:140,:] = image
			cv2.rectangle(cv_image, (0,120), (320,140), (0,0,0), 1)
			cv_image = cv2.resize(cv_image,(640,480))
			imgmsg = self.bridge.cv2_to_imgmsg(cv_image,"bgr8")
			self.pub_image_detection.publish(imgmsg)
		end = time.time()
		print('time cost in detect line: %.2f ms' %  ((end - start)*1000))
	def cbImg(self,cv_image):
		if self.rectify:
			cv_image = self.rector.rect(cv_image)
		if self.flip_code is not None:
			cv_image = cv2.flip(cv_image,self.flip_code)		
		self.cv_image = cv_image
		if self.pid_enabled:
			self.detectLine(cv_image)		
		image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		image_msg.header.stamp = rospy.Time.now()
		image_msg.header.frame_id = self.frame_id
		self.pub_raw.publish(image_msg)		
	def pidControl(self,event=None):
		if self.cv_image is None:
			return
		cv_image = self.cv_image
		if self.pid_enabled:
			self.detectLine(cv_image)
		# Publish raw image
		# cv_image = cv2.resize(cv_image,self.compress_size)

		# end = time.time()
	def srvGetImageTopics(self,params):
		topics = rospy.get_published_topics()
		res = GetStringsResponse()
		for topic in topics:
			if topic[1] == 'sensor_msgs/Image':
				res.data.append(topic[0])
		return res
	def cbPidSetEnable(self,params):
		if params.enabled == 1:
			print(params)
			self.pid_enabled = True
			self.base_speed = params.speed
			self.offset = params.offset
			self.factor = params.factor
		else:
			self.pid_enabled = False
			self.base_speed = 0
			self.car.setWheelsSpeed(0,0)
		return SetPidResponse("ok")
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
