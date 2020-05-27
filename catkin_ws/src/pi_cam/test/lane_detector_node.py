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

		# print(white.lines)
		white_c = len(white.lines)
		yellow_c = len(yellow.lines)
		red_c = len(red.lines)
		# white_lane = np.zeros(3)
		# yellow_lane = np.zeros(3)
		# red_lane = np.zeros(3)
		white_lane = [0,0,0]
		yellow_lane = [0,0,0]
		red_lane = [0,0,0]
		if white_c > 0:
			white_lane = self.detectLane2NoWeights(white)

		if yellow_c > 0 :	
			yellow_lane = self.detectLane2NoWeights(yellow)		
		if red_c > 0 :
			red_lane = self.detectLane2NoWeights(red)		

		self.intermittent_counter += 1
		# self.drive(white_lane,yellow_lane,red_lane)
		print('white d:%6f phi:%6f n:%3d , yellow d:%6f phi:%6f n:%3d , red d:%6f phi:%6f n:%3d ' % (white_lane[0],white_lane[1],
			white_c,yellow_lane[0],yellow_lane[1], yellow_c, red_lane[0],red_lane[1] , red_c))
		
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
	def detectLane2(self,detections):
		lines = np.array(detections.lines)*1.0
		n_lines = len(lines)
		points = np.zeros((n_lines*2,3))
		for i,line in enumerate(lines):
			x1,y1,x2,y2 = line
			vector = np.array([ x2 - x1, y2-y1] )
			weights = np.linalg.norm(vector)
			points[i*2] = x1,y1,weights
			points[i*2+1] = x2,y2,weights
		weights_total = points[:,2].sum()
		# print(lines)
		# print(points)
		x_m = (points[:,0]*points[:,2]).sum()/weights_total
		y_m = (points[:,1]*points[:,2]).sum()/weights_total
		m = ((points[:,0] - x_m)*(points[:,1]-y_m)*points[:,2]).sum()
		n = ((points[:,0] - x_m)*(points[:,0] - x_m)*points[:,2]).sum()
		a = m/n
		b = y_m - a * x_m
		phi = np.arctan(a)/np.pi*360
		d1 = (80*a-80+b)/np.linalg.norm(np.array([a,-1])) #画面底部中点到直线的距离
		if a == 0:
			d2 = 9999
		else:
			d2 = (80-b)/a -80 #直线与画面底部(y=80)的交点横坐标
		return  d2 , phi , n_lines
	def detectLane2NoWeights(self,detections):
		lines = np.array(detections.lines)*1.0
		n_lines = len(lines)
		points = np.zeros((n_lines*2,3))
		for i,line in enumerate(lines):
			x1,y1,x2,y2 = line
			vector = np.array([ x2 - x1, y2-y1] )
			weights = np.linalg.norm(vector)
			points[i*2] = x1,y1,weights/2
			points[i*2+1] = x2,y2,weights/2
		weights_total = points[:,2].sum()
		x_m = points[:,0].mean()
		y_m = points[:,1].mean()
		# print("x_m:%f y_m:%f" % (x_m,y_m))
		m = ((points[:,0] - x_m)*(points[:,1]-y_m)).sum()
		n = ((points[:,0] - x_m)*(points[:,0] - x_m)).sum()
		a = m/n
		b = y_m - a * x_m
		phi = np.arctan(a)/np.pi*360
		d1 = (80*a-80+b)/np.linalg.norm(np.array([a,-1])) #画面底部中点到直线的距离
		if a == 0:
			d2 = 9999
		else:
			d2 = (80-b)/a - 80 #直线与画面底部(y=80)的交点横坐标距中心的距离
		return  d2 , phi , n_lines
		# return b , a , n_lines

	def drive(self,white_lane,yellow_lane,red_lane):
		# return
		if white_lane[2] > 10 and yellow_lane[2] > 10:
			self.car.setWheelsSpeed(0.5,0.5)
		elif white_lane[2] > 10 and yellow_lane[2] < 10:
			self.car.setWheelsSpeed(0.4,0.7)
		elif white_lane[2] < 10 and yellow_lane[2] > 10:
			self.car.setWheelsSpeed(0.7,0.4)
		return		
		white_lane_mean = [0,0,0]
		yellow_lane_mean = [0,0,0]
		red_lane_mean = [0,0,0]
		if white_lane[2]>0:
			self.white_lane_his.append(white_lane)
			if len(self.white_lane_his)>=self.avg_num:
				self.white_lane_his = self.white_lane_his[-self.avg_num:]
			white_lane_mean = np.mean(np.array(self.white_lane_his),0)
		else:
			self.white_lane_his = []
		if yellow_lane[2]>0:
			self.yellow_lane_his.append(yellow_lane)
			if len(self.yellow_lane_his)>=self.avg_num:
				self.yellow_lane_his = self.yellow_lane_his[-self.avg_num:]
			yellow_lane_mean = np.mean(np.array(self.yellow_lane_his),0)
		else:
			self.yellow_lane_mean = []
		if red_lane[2]>0:
			self.red_lane_his.append(red_lane)
			if len(self.red_lane_his)>=self.avg_num:
				self.red_lane_his = self.red_lane_his[-self.avg_num:]
			red_lane_mean = np.mean(np.array(self.red_lane_his),0)
		else:
			self.red_lane_mean = []
		# print('white d:%6f phi:%6f n:%3d , yellow d:%6f phi:%6f n:%3d , red d:%6f phi:%6f n:%3d ' % (white_lane_mean[0],white_lane_mean[1],
			# white_lane_mean[2],yellow_lane_mean[0],yellow_lane_mean[1], yellow_lane_mean[2], red_lane_mean[0],red_lane_mean[1] , red_lane_mean[2]))

if __name__ == '__main__':
	rospy.init_node('lane_detector_node', anonymous=False)
	camera_node = LaneDetectorNode()
	rospy.on_shutdown(camera_node.onShutdown)
	rospy.spin()