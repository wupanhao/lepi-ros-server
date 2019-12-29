#!/usr/bin/python
#!coding:utf-8
# import the necessary packages
import time
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import FaceDetection
from pi_cam.srv import GetFaceDetections,GetFaceDetectionsResponse
from pi_driver.srv import GetStrings,GetStringsResponse,SetString,SetStringResponse
import os

from camera_utils import bgr_from_jpg
from face_recognizer import FaceRecognizer  

from pi_cam.srv import GetFrame,GetFrameRequest

# (0,10) => (-320,230)
# (320,240) => (0,0)
def toScratchAxes(cv_x,cv_y):
	return (cv_x-320,cv_y*-1+240)

class FaceRecognizerNode(object):
	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown." % (self.node_name))	
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.bridge = CvBridge()
		self.visualization = True

		self.image_msg = None
		self.pub_detections = rospy.Publisher("~image_face_detections", Image, queue_size=1)
		self.recognizer = FaceRecognizer(scale=3)

		rospy.Service('~detect_face_locations', GetFaceDetections, self.cbDetectFaceLocations)
		rospy.Service('~detect_face_labels', GetFaceDetections, self.cbDetectFaceLabels)
		rospy.Service('~list_face_labels', GetStrings, self.cbListFaceLabels)
		rospy.Service('~add_face_label', SetString, self.cbAddFaceLabel)
		rospy.Service('~remove_face_label', SetString, self.cbRemoveFaceLabel)
		# self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg , queue_size=1)
		# self.sub_image = rospy.Subscriber("~image_rect/compressed", CompressedImage, self.cbImg ,  queue_size=1)

		rospy.loginfo("[%s] wait_for_service : camera_get_frame..." % (self.node_name))
		rospy.wait_for_service('~camera_get_frame')
		self.get_frame = rospy.ServiceProxy('~camera_get_frame', GetFrame)
		rospy.loginfo("[%s] Initialized." % (self.node_name))

	def cbImg(self,image_msg):
		self.image_msg = image_msg

	def cbDetectFaceLocations(self,params):
		# print(params)
		res = self.get_frame(GetFrameRequest())
		self.image_msg = res.image			
		if self.image_msg == None:
			return GetFaceDetectionsResponse()
		if hasattr(self.image_msg,'format'): # CompressedImage
			try:
				cv_image = bgr_from_jpg(self.image_msg.data)
			except ValueError as e:
				rospy.loginfo('apriltag_detector cannot decode image: %s' % e)
				return GetFaceDetectionsResponse()
		else: # Image
			cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
		rect_image = cv_image
		face_locations = self.recognizer.detect(rect_image)
		if self.visualization:
			faces = self.recognizer.rect_faces(rect_image,face_locations)
			self.pub_detections.publish(self.bridge.cv2_to_imgmsg(faces,"bgr8"))
		return self.toFaceDetectionMsg(face_locations)
	def cbDetectFaceLabels(self,params):
		# print(params)
		res = self.get_frame(GetFrameRequest())
		self.image_msg = res.image			
		if self.image_msg == None:
			return GetFaceDetectionsResponse()
		if hasattr(self.image_msg,'format'): # CompressedImage
			try:
				cv_image = bgr_from_jpg(self.image_msg.data)
			except ValueError as e:
				rospy.loginfo('apriltag_detector cannot decode image: %s' % e)
				return GetFaceDetectionsResponse()
		else: # Image
			cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
		rect_image = cv_image
		face_locations,face_names = self.recognizer.recognize(rect_image)
		if self.visualization:
			faces = self.recognizer.label_faces(rect_image,face_locations,face_names)
			self.pub_detections.publish(self.bridge.cv2_to_imgmsg(faces,"bgr8"))		
		return self.toFaceDetectionMsg(face_locations,face_names)
	def toFaceDetectionMsg(self,face_locations=[],face_names=[]):
		msg = GetFaceDetectionsResponse()
		if len(face_names)>0 and len(face_names) == len(face_locations):
			for (top, right, bottom, left), name in zip(face_locations, face_names):
				cv_x = (right+left)/2*self.recognizer.scale
				cv_y = (top+bottom)/2*self.recognizer.scale
				scratch_x,scratch_y = toScratchAxes(cv_x,cv_y)
				width = right - left
				height = bottom - top
				face_detection = FaceDetection(name,[scratch_x,scratch_y,width,height])
				msg.detections.append(face_detection)
		elif len(face_locations) > 0:
			for (top, right, bottom, left) in face_locations:
				cv_x = (right+left)/2*self.recognizer.scale
				cv_y = (top+bottom)/2*self.recognizer.scale
				scratch_x,scratch_y = toScratchAxes(cv_x,cv_y)
				width = right - left
				height = bottom - top
				face_detection = FaceDetection("",[scratch_x,scratch_y,width,height])
				msg.detections.append(face_detection)
		return msg
	def cbListFaceLabels(self,params):
		return GetStringsResponse(self.recognizer.known_faces.keys())
	def cbAddFaceLabel(self,params):
		if len(params.data) == 0:
			return SetStringResponse("添加失败,名称长度为0")
		try:
			cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
			res = self.recognizer.add_face_label(cv_image,params.data,save=True)
			return SetStringResponse(res)	
		except Exception as e:
			print(e)
			return SetStringResponse("添加失败")
	def cbRemoveFaceLabel(self,params):
		if len(params.data) == 0:
			return SetStringResponse("未提供要删除的标签")
		try:
			res = self.recognizer.remove_face_label(params.data)
			return SetStringResponse(res)
		except Exception as e:
			print(e)
			return SetStringResponse("删除失败")		
if __name__ == '__main__':
	rospy.init_node('face_recognizer_node', anonymous=False)
	node = FaceRecognizerNode()
	rospy.on_shutdown(node.onShutdown)
	rospy.spin()
