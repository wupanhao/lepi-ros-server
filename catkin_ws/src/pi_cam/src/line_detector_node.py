#!/usr/bin/python
#!coding:utf-8
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from line_detector import LineDetector
from pi_cam.srv import GetLineDetection,GetLineDetectionResponse
class LineDetectorNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.bridge = CvBridge()
		self.cv_image = None
		self.detector = LineDetector()
		self.visualization = True

		# self.config_file = os.path.dirname(os.path.abspath(__file__)) + "/default.yaml"
		self.image_msg = None
		self.pub_detections = rospy.Publisher("~image_detections", Image, queue_size=1)

		self.tag_srv = rospy.Service('~detect_line', GetLineDetection, self.cbGetLineDetection)
		self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
		# self.sub_image = rospy.Subscriber("~image_rect/compressed", CompressedImage, self.cbImg ,  queue_size=1)
		rospy.loginfo("[%s] Initialized." % (self.node_name))

	def cbImg(self,image_msg):
		self.image_msg = image_msg

	def cbGetLineDetection(self,params):
		# print(params)
		if self.image_msg == None:
			print('[%s]: camera msg not received' % (self.node_name))
			return GetLineDetectionResponse()
		if hasattr(self.image_msg,'format'): # CompressedImage
			try:
				cv_image = bgr_from_jpg(self.image_msg.data)
			except ValueError as e:
				rospy.loginfo('apriltag_detector cannot decode image: %s' % e)
				return
		else: # Image
			cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
		rect_image = cv_image
		cnt,image = self.detector.detect_color(rect_image,self.detector.colors[u'黄线'])
		if self.visualization:
			imgmsg = self.bridge.cv2_to_imgmsg(image,"bgr8")
			self.pub_detections.publish(imgmsg)			
		return self.toLineDetections(cnt)
	def toLineDetections(self,cnt):
		if cnt is not None:
			center,wh,angle = cv2.minAreaRect(cnt)
			return GetLineDetectionResponse(center,wh,angle)
		return GetLineDetectionResponse()

	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
	rospy.init_node('line_detector_node', anonymous=False)
	node = LineDetectorNode()
	rospy.on_shutdown(node.onShutdown)
	rospy.spin()
