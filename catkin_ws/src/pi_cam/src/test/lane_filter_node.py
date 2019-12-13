#!coding:utf-8
import rospy
import rospkg
import cv2
import numpy as np
from line_detector import LineDetectorHSV
from line_detector.line_detector_plot import color_segment, drawLines

from cv_bridge import CvBridge
import threading
import yaml
from sensor_msgs.msg import Image,CompressedImage

from duckietown_msgs.msg import Segment,SegmentList

from Car import CarDriver

class LaneDetectorNode:
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))


		self.bridge = CvBridge()
		# Thread lock
		self.thread_lock = threading.Lock()

		self.active = True
		self.verbose = True

		self.car = CarDriver()
		self.white_lane_his = []
		self.yellow_lane_his = []
		self.red_lane_his = []		

		self.avg_num = 5

		# self.stats = Stats()

		# Only be verbose every 10 cycles
		self.intermittent_interval = 5
		self.intermittent_counter = 0

		fstream = file(rospkg.RosPack().get_path('line_detector')+'/include/line_detector/config/default.yaml', 'r')
		configuration = yaml.load(fstream)

		self.detector = LineDetectorHSV(configuration['detector'][1]['configuration'])
		self.detector_used = self.detector
		self.image_size = [120,160]
		self.top_cutoff = 40
		self.sub_rect = rospy.Subscriber("/camera_node/image_rect", Image, self.cbImage, queue_size=1)
		self.pub_image = rospy.Publisher("~image_with_lines",Image,queue_size=1)
		self.pub_edge = rospy.Publisher("~image_with_edge",Image,queue_size=1)
		self.pub_colorSegment = rospy.Publisher("~image_color_segment",Image,queue_size=1)
		self.pub_lines = rospy.Publisher("~segment_list", SegmentList, queue_size=1)
		rospy.loginfo("[%s] Initialized." % (self.node_name))

	def cbImage(self,image_msg):
		# self.sub_rect.unregister()
		image_cv = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
		hei_original, wid_original = image_cv.shape[0:2]

		if self.image_size[0] != hei_original or self.image_size[1] != wid_original:
			# image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
			image_cv = cv2.resize(image_cv, (self.image_size[1], self.image_size[0]),
								   interpolation=cv2.INTER_NEAREST)
		image_cv = image_cv[self.top_cutoff:,:,:]

		# Set the image to be detected
		self.detector_used.setImage(image_cv)

		# Detect lines and normals
		white = self.detector_used.detectLines('white')
		yellow = self.detector_used.detectLines('yellow')
		red = self.detector_used.detectLines('red')

		# SegmentList constructor
		segmentList = SegmentList()
		segmentList.header.stamp = image_msg.header.stamp

		# Convert to normalized pixel coordinates, and add segments to segmentList
		arr_cutoff = np.array((0, self.top_cutoff, 0, self.top_cutoff))
		arr_ratio = np.array((1./self.image_size[1], 1./self.image_size[0], 1./self.image_size[1], 1./self.image_size[0]))
		if len(white.lines) > 0:
			lines_normalized_white = ((white.lines + arr_cutoff) * arr_ratio)
			segmentList.segments.extend(self.toSegmentMsg(lines_normalized_white, white.normals, Segment.WHITE))
		if len(yellow.lines) > 0:
			lines_normalized_yellow = ((yellow.lines + arr_cutoff) * arr_ratio)
			segmentList.segments.extend(self.toSegmentMsg(lines_normalized_yellow, yellow.normals, Segment.YELLOW))
		if len(red.lines) > 0:
			lines_normalized_red = ((red.lines + arr_cutoff) * arr_ratio)
			segmentList.segments.extend(self.toSegmentMsg(lines_normalized_red, red.normals, Segment.RED))

		rospy.loginfo('# segments: white %3d yellow %3d red %3d' % (len(white.lines),
				len(yellow.lines), len(red.lines)))

		# Publish segmentList
		self.pub_lines.publish(segmentList)

		
		if self.verbose:
			# print('line_detect_node: verbose is on!')

			# Draw lines and normals
			image_with_lines = np.copy(image_cv)
			drawLines(image_with_lines, white.lines, (0, 0, 0))
			drawLines(image_with_lines, yellow.lines, (255, 0, 0))
			drawLines(image_with_lines, red.lines, (0, 255, 0))

			# Publish the frame with lines
			image_msg_out = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
			image_msg_out.header.stamp = image_msg.header.stamp
			self.pub_image.publish(image_msg_out)

			colorSegment = color_segment(white.area, red.area, yellow.area)
			edge_msg_out = self.bridge.cv2_to_imgmsg(self.detector_used.edges, "mono8")
			colorSegment_msg_out = self.bridge.cv2_to_imgmsg(colorSegment, "bgr8")
			self.pub_edge.publish(edge_msg_out)
			self.pub_colorSegment.publish(colorSegment_msg_out)
	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown." % (self.node_name))
	def toSegmentMsg(self,  lines, normals, color):

		segmentMsgList = []
		for x1,y1,x2,y2,norm_x,norm_y in np.hstack((lines,normals)):
			segment = Segment()
			segment.color = color
			segment.pixels_normalized[0].x = x1
			segment.pixels_normalized[0].y = y1
			segment.pixels_normalized[1].x = x2
			segment.pixels_normalized[1].y = y2
			segment.normal.x = norm_x
			segment.normal.y = norm_y

			segmentMsgList.append(segment)
		return segmentMsgList
if __name__ == '__main__':
	rospy.init_node('lane_detector_node', anonymous=False)
	camera_node = LaneDetectorNode()
	rospy.on_shutdown(camera_node.onShutdown)
	rospy.spin()