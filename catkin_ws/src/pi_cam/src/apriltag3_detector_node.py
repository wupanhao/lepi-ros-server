#!/usr/bin/python
#!coding:utf-8
# import the necessary packages
import time
import cv2
from cv_bridge import CvBridge
import rospkg
import rospy
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np
from camera_utils import load_camera_info_2
from pi_cam.msg import ApriltagPose
from pi_cam.srv import GetApriltagDetections,GetApriltagDetectionsResponse
from apriltag_detector import ApriltagDetector
from image_rector import ImageRector
from scipy.spatial.transform import Rotation as R
def bgr_from_jpg(data):
	""" Returns an OpenCV BGR image from a string """
	s = np.fromstring(data, np.uint8)
	bgr = cv2.imdecode(s, cv2.IMREAD_COLOR)
	if bgr is None:
		msg = 'Could not decode image (cv2.imdecode returned None). '
		msg += 'This is usual a sign of data corruption.'
		raise ValueError(msg)
	return bgr
class ApriltagDetectorNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.DIM = (640, 480)
		self.bridge = CvBridge()
		self.cv_image = None
		# allow the camera to warmup
		time.sleep(0.1)
		self.rector = ImageRector()
		self.detector = ApriltagDetector()
		self.visualization = True

		self.tag_detect_rate = 4

		self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		self.image_msg = None
		self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)
		self.pub_rect = rospy.Publisher("~image_rect", Image, queue_size=1)

		self.pub_detections = rospy.Publisher("~image_detections", Image, queue_size=1)

		# self.sub_compressed = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
		self.sub_compressed = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.cbImg ,  queue_size=1)

		self.tag_srv = rospy.Service('~detect_apriltag', GetApriltagDetections, self.cbGetApriltagDetections)

	def cbImg(self,image_msg):
		self.image_msg = image_msg

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
			cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
		self.pub_raw.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		# rect_image = cv_image
		rect_image = self.rector.rect(cv_image)
		self.pub_rect.publish(self.bridge.cv2_to_imgmsg(rect_image,"bgr8"))
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
	rospy.init_node('apriltag_detector_node', anonymous=False)
	node = ApriltagDetectorNode()
	rospy.on_shutdown(node.onShutdown)
	rospy.spin()
