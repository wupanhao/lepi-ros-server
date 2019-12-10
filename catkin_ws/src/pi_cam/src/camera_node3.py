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
from pi_cam.msg import ApriltagPose
from pi_cam.srv import GetApriltagDetections,GetApriltagDetectionsResponse
from apriltag_detector import ApriltagDetector
from image_rector import ImageRector
from scipy.spatial.transform import Rotation as R
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
		self.camera.rotation = 0
		self.cv_image = None
		# allow the camera to warmup
		time.sleep(0.1)
		self.r = rospy.Rate(self.rate)
		self.rector = ImageRector()
		self.detector = ApriltagDetector()
		self.visualization = True

		self.tag_detect_rate = 4

		self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		self.camera_info_msg_rect = load_camera_info_2(self.cali_file)
		self.image_msg = Image()
		self.rawCapture = PiRGBArray(self.camera, size=self.DIM)
		self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)
		self.pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)

		self.pub_detections = rospy.Publisher("~image_detections", Image, queue_size=1)

		self.stream = io.BytesIO()
		self.pub_compressed = rospy.Publisher("~image_raw/compressed", CompressedImage, queue_size=1)

		self.pub_rect = rospy.Publisher("~rect/image", Image, queue_size=1)
		self.pub_camera_info_rect = rospy.Publisher("~rect/camera_info", CameraInfo, queue_size=1)

		self.tag_srv = rospy.Service('~detect_apriltag', GetApriltagDetections, self.cbGetApriltagDetections)

		self.timer_init = rospy.Timer(rospy.Duration.from_sec(1.0/self.tag_detect_rate), self.publish_rect)

		# frame = next( self.camera.capture_continuous(self.stream, format="jpeg", use_video_port=True) )
		# print(frame)

	def startCaptureRawCV(self):
		while not self.is_shutdown and not rospy.is_shutdown():
			frame = next( self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True) )
			self.cv_image = frame.array

			image_msg = self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8")
			image_msg.header.stamp = rospy.Time.now()
			image_msg.header.frame_id = self.frame_id
			self.image_msg = image_msg
			self.pub_raw.publish(image_msg)			

			self.camera_info_msg.header = image_msg.header
			self.pub_camera_info.publish(self.camera_info_msg)

			'''
			rect_image = self.rector.rect(self.cv_image)
			rect_image_msg = self.bridge.cv2_to_imgmsg(rect_image, "bgr8")
			rect_image_msg.header = image_msg.header
			self.pub_rect.publish(rect_image_msg)			
			'''

			# self.r.sleep()
			self.rawCapture.truncate(0)
		self.camera.close()
	def publish_rect(self,event):
		if self.cv_image == None:
			return
		rect_image = self.rector.rect(self.cv_image)
		rect_msg = self.bridge.cv2_to_imgmsg(rect_image,"bgr8")
		rect_msg.header.stamp = rospy.Time.now()
		self.camera_info_msg_rect.header = rect_msg.header
		self.pub_camera_info_rect.publish(self.camera_info_msg_rect)
		self.pub_rect.publish(rect_msg)
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
			# self.decodeAndPublishRaw(image_msg)
			# self.r.sleep()
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
	def cbGetApriltagDetections(self,params):
		# print(params)
		if self.image_msg == None:
			return GetApriltagDetectionsResponse()
		if hasattr(self.image_msg,'format'): # CompressedImage
			try:
				cv_image = bgr_from_jpg(self.image_msg.data)
			except ValueError as e:
				rospy.loginfo('Anti_instagram cannot decode image: %s' % e)
				return
		else: # Image
			cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
			self.pub_raw.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			rect_image = self.rector.rect(self.cv_image)
			# self.pub_rect.publish(self.bridge.cv2_to_imgmsg(rect_image,"bgr8"))
			tags = self.detector.detect(rect_image)
			if self.visualization:
				for tag in tags:
					for idx in range(len(tag.corners)):
						cv2.line(rect_image, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
					# cv2.putText(rect_image, "中文测试", # not work
					cv2.putText(rect_image, str(tag.tag_id),
								org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
								fontFace=cv2.FONT_HERSHEY_SIMPLEX,
								fontScale=0.8,
								color=(0, 0, 255))
				imgmsg = self.bridge.cv2_to_imgmsg(rect_image,"bgr8")
				self.pub_detections.publish(imgmsg)			
			return self.toApriltagDetections(tags)
		return 
	def toApriltagDetections(self,tags):
		msg = GetApriltagDetectionsResponse()
		for tag in tags:
			r = R.from_dcm(np.array(tag.pose_R))
			offset = np.array(tag.pose_t)*100
			euler = r.as_euler('xyz', degrees=True)			
			detection = ApriltagPose(id=tag.tag_id,pose_r=euler,pose_t=offset)
			msg.detections.append(detection)
		return msg
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
