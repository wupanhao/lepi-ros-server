#!/usr/bin/python
#!coding:utf-8
# import the necessary packages
import time
#!coding:utf-8
# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py
import cv2
from cv_bridge import CvBridge
import rospkg
import rospy
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np
from camera_utils import load_camera_info_2
from pi_cam.msg import ApriltagPose

from pi_cam.srv import GetApriltagDetections,GetApriltagDetectionsResponse,GetPredictions,GetPredictionsResponse
from apriltag_detector import ApriltagDetector
# from image_rector import ImageRector
from scipy.spatial.transform import Rotation as R

from pi_driver.srv import SetInt32,SetInt32Response,GetInt32,GetInt32Response
from pi_driver.srv import SetString,SetStringResponse,GetStrings,GetStringsResponse
from usb_camera import UsbCamera
from keras_transfer import ImageClassifier,AccuracyLogger
import os
from std_msgs.msg import String
from data_utils import get_labels

class CameraNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.DIM = (640, 480)
		self.rate = 15
		self.bridge = CvBridge()
		self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
		self.camera = UsbCamera(callback = self.PublishRaw)
		self.ic = ImageClassifier()
		# self.ic.load_model('/root/keras/分类测试/model.h5')
		self.data_root = self.ic.data_root
		self.ns = None
		self.cv_image = None
		# allow the camera to warmup
		time.sleep(0.1)
		self.r = rospy.Rate(self.rate)
		# self.rector = ImageRector()
		self.detector = ApriltagDetector()
		self.visualization = True

		self.tag_detect_rate = 4
		self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		self.camera_info_msg_rect = load_camera_info_2(self.cali_file)
		self.image_msg = None # Image()
		self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)
		self.pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)

		self.pub_detections = rospy.Publisher("~image_detections", Image, queue_size=1)

		self.training_logs_topic = rospy.Publisher("~training_logs", String, queue_size=1)

		# self.pub_compressed = rospy.Publisher("~image_raw/compressed", CompressedImage, queue_size=1)
		# self.pub_rect = rospy.Publisher("~rect/image", Image, queue_size=1)
		# self.pub_camera_info_rect = rospy.Publisher("~rect/camera_info", CameraInfo, queue_size=1)

		rospy.Service('~detect_apriltag', GetApriltagDetections, self.cbGetApriltagDetections)
		rospy.Service('~camera_set_enable', SetInt32, self.srvCameraSetEnable)
		rospy.Service('~camera_save_frame', SetString, self.srvCameraSaveFrame)
		rospy.Service('~set_ns', SetString, self.srv_set_ns)
		rospy.Service('~delete_ns', SetString, self.srv_delete_ns)
		rospy.Service('~create_cat', SetString, self.srv_create_cat)
		rospy.Service('~delete_cat', SetString, self.srv_delete_cat)
		rospy.Service('~list_ns', SetString, self.srv_list_ns)
		rospy.Service('~list_cat', SetString, self.srv_list_cat)
		rospy.Service('~train_classifier', SetString, self.srv_train_classifier)
		rospy.Service('~predict', GetPredictions, self.srv_predict)
		rospy.Service('~get_training_data', GetPredictions, self.srv_get_training_data)

		# self.timer_init = rospy.Timer(rospy.Duration.from_sec(1.0/self.tag_detect_rate), self.publish_rect)
		rospy.loginfo("[%s] Initialized......" % (self.node_name))
	def srvCameraSetEnable(self,params):
		if params.value == 1 and self.camera.active == False:
			ret = self.camera.open_camera(params.port)
			return SetInt32Response(0,ret)
		elif params.value == 0:
			self.camera.active = False
			return SetInt32Response(0,0)
		else:
			return SetInt32Response(1,1)
	def PublishRaw(self,cv_image):
		# Publish raw image
		self.cv_image = cv_image
		img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		# time_2 = time.time()
		img_msg.header.stamp = img_msg.header.stamp
		img_msg.header.frame_id = img_msg.header.frame_id
		self.pub_raw.publish(img_msg)
		self.image_msg = img_msg
	def srvCameraSaveFrame(self,params):
		if self.ns is None:
			return SetStringResponse("训练没有定义，创建或者选择一次训练")
		try:
			cat_name = params.data
			directory = os.path.join(self.data_root, self.ns,cat_name)
			file_name = '%d.jpg' % (len(os.listdir(directory))+1)
			full_path = os.path.join(directory,file_name)
			if self.cv_image is not None:
				# cv_image = self.bridge.imgmsg_to_cv2(self.img_msg, desired_encoding="bgr8")
				cv2.imwrite(full_path,self.cv_image) # not working
				# print(self.cv_image)
				# cv2.imencode('.jpg', self.cv_image)[1].tofile(full_path) # 正确方法
				return SetStringResponse("保存至%s成功" % (full_path))
			else:
				return SetStringResponse("保存出错")
		except Exception as e:
			print(e)
			return SetStringResponse("保存出错")
		finally:
			pass
	def srv_create_cat(self,params):
		if self.ns == None:
			return SetStringResponse("请先设置训练名称")
		try:
			data_dir = os.path.join(self.data_root,self.ns,params.data)
			if os.path.exists(data_dir):
				print('data_dir %s exists' %(data_dir))
				return SetStringResponse("分类已存在")
			else:
				print('create data_dir %s' %(data_dir))
				os.makedirs(data_dir)
				return SetStringResponse("创建成功")
		except Exception as e:
			print(e)
			return SetStringResponse("创建失败")

	def srv_list_cat(self,params):
		data_dir = os.path.join(self.data_root,self.ns)
		return GetStringsResponse(os.listdir(data_dir))
	def srv_list_ns(self,params):
		return GetStringsResponse(os.listdir(self.data_root))
	def srv_set_ns(self,params):
		try:
			data_dir = os.path.join(self.data_root,params.data)
			if not os.path.exists(data_dir):		
				os.makedirs(data_dir)
			self.ns = params.data
			self.ic.ns = params.data
			return SetStringResponse("设置成功")
		except Exception as e:
			print(e)
			return SetStringResponse("设置失败")
	def srv_delete_ns(self,params):
		try:
			data_dir = os.path.join(self.data_root,params.data)
			if len(data_dir)>len(self.data_root):
				os.system('rm -rf '+data_dir)
				return SetStringResponse("删除成功")
		except Exception as e:
			print(e)
			return SetStringResponse("删除失败")
	def srv_delete_cat(self,params):
		if self.ns is None:
			return SetStringResponse("请先设置训练名称")
		try:
			data_dir = os.path.join(self.data_root,self.ns,params.data)
			if len(data_dir)>len(self.data_root):
				os.system('rm -rf '+data_dir)
				return SetStringResponse("删除成功")
		except Exception as e:
			print(e)
			return SetStringResponse("删除失败")
	def srv_train_classifier(self,params):
		try:
			epochs = int(params.data)
			if epochs <= 0:
				return SetStringResponse('至少训练1次~')
			data_dir = os.path.join(self.data_root,self.ns)
			self.epochs = epochs
			self.ic.ns = self.ns
			self.ic.train(data_dir,epochs,self.pub_training_logs)
			return SetStringResponse('训练完成，可以进行测试了')
		except Exception as e:
			print(e)
			return SetStringResponse('训练失败')
	def srv_predict(self,params):
		if self.ic.model is None:
			return GetPredictionsResponse()
		try:
			res = self.ic.predict(cv_img = self.cv_image)
			if res is not None:
				return GetPredictionsResponse(res[0],res[1])
		except Exception as e:
			print(e)
			return GetPredictionsResponse()
	def pub_training_logs(self,epoch,batch,logs):
		# logs {'loss': 0.33773628, 'accuracy': 0.71428573, 'batch': 6, 'size': 4}
		self.training_logs_topic.publish(String('epoch: %d/%d,batch: %d,loss: %.2f, accuracy: %.2f' % (epoch+1,self.epochs,batch,logs['loss'],logs['accuracy'])))
	def srv_get_training_data(self,params):
		data_dir = os.path.join(self.data_root,self.ns)
		labels = get_labels(data_dir)
		counts = [len(os.listdir(os.path.join(data_dir,label))) for label in labels]
		return GetPredictionsResponse(labels,counts)
	def cbGetApriltagDetections(self,params):
		# print(params)
		if self.image_msg == None:
			return GetApriltagDetectionsResponse()
		if hasattr(self.image_msg,'format'): # CompressedImage
			try:
				cv_image = bgr_from_jpg(self.image_msg.data)
			except ValueError as e:
				rospy.loginfo('bgr_from_jpg: %s' % e)
				return
		else: # Image
			cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
			# self.pub_raw.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
			# rect_image = self.rector.rect(self.cv_image)
			rect_image = cv_image
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
	# thread.start_new_thread(camera_node.startCaptureCompressed, ())
	rospy.spin()
