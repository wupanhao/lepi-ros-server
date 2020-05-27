#!/usr/bin/python
#!coding:utf-8
import time
# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np

from pi_cam.srv import GetPredictions,GetPredictionsResponse
# from pi_driver.srv import GetInt32,GetInt32Response
from pi_driver.srv import SetInt32,SetInt32Response
from pi_driver.srv import SetString,SetStringResponse,GetStrings,GetStringsResponse
from keras_transfer import ImageClassifier,AccuracyLogger
import os
from std_msgs.msg import String
from data_utils import get_labels
from pi_cam.srv import GetFrame,GetFrameRequest
from std_msgs.msg import Empty

class TransferNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))

		self.is_shutdown = False
		self.bridge = CvBridge()
		self.ic = ImageClassifier()
		# self.ic.load_model('/root/keras/分类测试/model.h5')
		self.data_root = self.ic.data_root
		self.ns = None
		self.image_msg = None # Image()

		self.training_logs_topic = rospy.Publisher("~training_logs", String, queue_size=1)
		self.pub_image = rospy.Publisher("~image_transfer", Image, queue_size=1)

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
		self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
		rospy.Subscriber('~shutdown', Empty, self.cbShutdown)
		rospy.loginfo("[%s] wait_for_service : camera_get_frame..." % (self.node_name))
		rospy.wait_for_service('~camera_get_frame')
		self.get_frame = rospy.ServiceProxy('~camera_get_frame', GetFrame)
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
	def getBox(self):
		img_size = self.ic.IMAGE_SIZE
		xmin = (480 - img_size)/2
		ymin = (360 - img_size)/2
		xmax = xmin + img_size
		ymax = ymin + img_size
		return [xmin,ymin,xmax,ymax]
	def cbImg(self,image_msg):
		self.image_msg = image_msg
		cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
		xmin,ymin,xmax,ymax = self.getBox()
		cv2.rectangle(cv_image,(xmin,ymin),(xmax,ymax),(10, 255, 0), 2)
		msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
		self.pub_image.publish(msg)
	def srvCameraSaveFrame(self,params):
		if self.ns is None:
			return SetStringResponse("训练没有定义，创建或者选择一次训练")
		try:
			res = self.get_frame(GetFrameRequest())
			self.image_msg = res.image		
			cat_name = params.data
			directory = os.path.join(self.data_root, self.ns,cat_name)
			file_name = '%d.jpg' % (len(os.listdir(directory))+1)
			full_path = os.path.join(directory,file_name)
			if self.image_msg is not None:
				cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
				xmin,ymin,xmax,ymax = self.getBox()
				cv_image = cv_image[ymin:ymax,xmin:xmax]
				cv2.imwrite(full_path,cv_image)
				# print(cv_image)
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
		if self.ns is not None:
			data_dir = os.path.join(self.data_root,self.ns)
			return GetStringsResponse(os.listdir(data_dir))
		else:
			return GetStringsResponse()
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
		res = self.get_frame(GetFrameRequest())
		self.image_msg = res.image
		if self.ic.model is None or self.image_msg is None:
			return GetPredictionsResponse()
		try:
			cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")	
			xmin,ymin,xmax,ymax = self.getBox()
			cv_image = cv_image[ymin:ymax,xmin:xmax]
			res = self.ic.predict(cv_img = cv_image)
			if res is not None:
				return GetPredictionsResponse(res[0],res[1])
		except Exception as e:
			print(e)
			return GetPredictionsResponse()
	def pub_training_logs(self,epoch,batch,logs):
		# logs {'loss': 0.33773628, 'accuracy': 0.71428573, 'batch': 6, 'size': 4}
		print(logs)
		msg = String('第%d/%d轮, 批次: %d, 损失: %.2f, 准确率: %.2f' % (epoch+1,self.epochs,batch,logs['loss'],logs['acc']))
		self.training_logs_topic.publish(msg)
	def srv_get_training_data(self,params):
		data_dir = os.path.join(self.data_root,self.ns)
		labels = get_labels(data_dir)
		counts = [len(os.listdir(os.path.join(data_dir,label))) for label in labels]
		return GetPredictionsResponse(labels,counts)
	def cbShutdown(self,msg):
		rospy.loginfo("[%s] receiving shutdown msg." % (self.node_name))
		rospy.signal_shutdown("shutdown receiving msg")
	def onShutdown(self):
		rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
	rospy.init_node('transfer_learning_node', anonymous=False)
	node = TransferNode()
	rospy.on_shutdown(node.onShutdown)
	rospy.spin()
