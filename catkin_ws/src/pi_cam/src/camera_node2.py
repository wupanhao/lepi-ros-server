#!/usr/bin/python
#!coding:utf-8
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from cv_bridge import CvBridge
import rospkg
import rospy
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import io
import numpy as np
import thread
from camera_utils import load_camera_info_2
from image_rector import ImageRector

class CameraNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.DIM = (640, 480)
		self.rate = 15
		self.bridge = CvBridge()
		self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
		# initialize the camera and grab a reference to the raw camera capture
		self.camera = PiCamera()
		self.camera.resolution = self.DIM		
		self.camera.framerate = self.rate
		self.camera.rotation = 180
		self.cv_image = None
		# allow the camera to warmup
		time.sleep(0.1)
		self.rector = ImageRector()
		self.visualization = True

		self.tag_detect_rate = 4

		self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		self.image_msg = Image()
		self.rawCapture = PiRGBArray(self.camera, size=self.DIM)
		self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)
		self.pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)

		self.stream = io.BytesIO()
		self.pub_compressed = rospy.Publisher("~image_raw/compressed", CompressedImage, queue_size=1)

		self.pub_camera_info_rect = rospy.Publisher("~rect/camera_info", CameraInfo, queue_size=1)

	def startCaptureCompressed(self):
		while not self.is_shutdown and not rospy.is_shutdown():
			gen = self.genCaptureCompressed()
			self.camera.capture_sequence(gen, format="jpeg", use_video_port=True, splitter_port=0)
		self.camera.close()

	def genCaptureCompressed(self):
		while not self.is_shutdown and not rospy.is_shutdown():
			yield self.stream
			self.stream.seek(0)
			image_msg = CompressedImage()
			image_msg.format = "jpeg"
			image_msg.data = self.stream.getvalue()
			image_msg.header.stamp = rospy.Time.now()
			image_msg.header.frame_id = self.frame_id
			self.image_msg = image_msg
			self.pub_compressed.publish(image_msg)			
			self.decodeAndPublishRaw(image_msg)
			self.stream.seek(0)
			self.stream.truncate()
	def decodeAndPublishRaw(self,image_msg):
		# Publish raw image
		np_arr = np.fromstring(image_msg.data, np.uint8)
		cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		# time_1 = time.time()
		img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		# time_2 = time.time()
		img_msg.header.stamp = image_msg.header.stamp
		img_msg.header.frame_id = image_msg.header.frame_id
		self.pub_raw.publish(img_msg)

	def onShutdown(self):
		rospy.loginfo("[%s] Closing camera." % (self.node_name))
		self.is_shutdown = True
		rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
	rospy.init_node('camera_node', anonymous=False)
	camera_node = CameraNode()
	rospy.on_shutdown(camera_node.onShutdown)
	#thread.start_new_thread(camera_node.startCaptureRawCV, ())
	thread.start_new_thread(camera_node.startCaptureCompressed, ())
	rospy.spin()
