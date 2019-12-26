#!/usr/bin/python
#!coding:utf-8
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from line_detector import LineDetector
from pi_cam.srv import GetLineDetection,GetLineDetectionResponse,GetFrame,GetFrameRequest
from pi_cam.msg import LineDetection
import time
class LineDetectorNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.bridge = CvBridge()
		self.cv_image = None
		self.detector = LineDetector()
		self.visualization = True
		self.size = (320,240)

		# self.config_file = os.path.dirname(os.path.abspath(__file__)) + "/default.yaml"
		self.image_msg = None
		self.line_msg = GetLineDetectionResponse()
		self.pub_image_detection = rospy.Publisher("~image_detections", Image, queue_size=1)
		self.pub_line_detection = rospy.Publisher("~line_detection", LineDetection, queue_size=1)

		self.tag_srv = rospy.Service('~detect_line', GetLineDetection, self.cbGetLineDetection)
		self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,queue_size=1)
		# self.sub_image = rospy.Subscriber("~image_rect/compressed", CompressedImage, self.cbImg ,queue_size=1)

		rospy.loginfo("[%s] wait_for_service : camera_get_frame..." % (self.node_name))
		rospy.wait_for_service('~camera_get_frame')
		self.get_frame = rospy.ServiceProxy('~camera_get_frame', GetFrame)
		rospy.loginfo("[%s] Initialized." % (self.node_name))

	def cbImg(self,image_msg):
		self.image_msg = image_msg
		self.line_msg = self.detectLine()
		self.pub_line_detection.publish(self.line_msg)
	def detectLine(self,params=None):
		try:
			start = time.time()
			# res = self.get_frame(GetFrameRequest())
			# res = self.get_frame(GetFrameRequest([320,240]))
			# self.image_msg = res.image
			# end = time.time()
			# print('time cost in get image frame from service : %.2f ms' % ((end - start)*1000))
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

		# print(params)
		if self.image_msg == None:
			print('[%s]: camera msg not received' % (self.node_name))
			return GetLineDetectionResponse()
		if hasattr(self.image_msg,'format'): # CompressedImage
			try:
				cv_image = bgr_from_jpg(self.image_msg.data)
			except ValueError as e:
				rospy.loginfo('[%s] cannot decode image: %s' % (self.node_name,e))
				return GetLineDetectionResponse()
		else: # Image
			try:
				cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
			except Exception as e:
				rospy.loginfo('[%s] cannot convert image: %s' % (self.node_name,e))
				return GetLineDetectionResponse()
		if cv_image.shape[0]!=self.size[1] or cv_image.shape[1]!=self.size[0]:
			cv_image = cv2.resize(cv_image,self.size)
		rect_image = cv_image[120:140,:]
		cnt,image = self.detector.detect_color(rect_image,self.detector.colors[u'黄线'])
		if self.visualization:
			imgmsg = self.bridge.cv2_to_imgmsg(image,"bgr8")
			self.pub_image_detection.publish(imgmsg)
		end = time.time()
		# print('time cost in detect line: %.2f ms' %  ((end - start)*1000))
		return self.toLineDetections(cnt)
	def toLineDetections(self,cnt):
		if cnt is not None:
			center,wh,angle = cv2.minAreaRect(cnt)
			return GetLineDetectionResponse(center,wh,angle)
		return GetLineDetectionResponse()
	def cbGetLineDetection(self,params):
		return self.line_msg
	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
	rospy.init_node('line_detector_node', anonymous=False)
	node = LineDetectorNode()
	rospy.on_shutdown(node.onShutdown)
	rospy.spin()
