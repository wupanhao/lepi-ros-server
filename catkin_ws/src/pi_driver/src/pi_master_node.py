#!/usr/bin/python
#!coding:utf-8

import rospy
import rosnode
import os
import time
from pi_driver.srv import GetString,GetStringResponse
from pi_driver.srv import GetStrings,GetStringsResponse
from pi_driver.srv import SetString,SetStringResponse
from pi_driver.srv import GetInt32,GetInt32Response
from std_msgs.msg import String

ignores = ['/rosout','/rosapi','/web_video_server_1','/ubiquityrobot/pi_driver_node',
	'/rosbridge_websocket','/ubiquityrobot/pi_master_node']

available_nodes = [
	'/ubiquityrobot/camera_node',
	'/ubiquityrobot/apriltag_detector_node',
	'/ubiquityrobot/transfer_learning_node',
	'/ubiquityrobot/line_detector_node',
	'/ubiquityrobot/face_recognizer_node',
	'/ubiquityrobot/joystick_node',
		]

launch_cmds = [
	'roslaunch pi_cam camera_node.launch > /tmp/camera_node.log &',
	'roslaunch pi_cam apriltag_detector_node.launch > /tmp/apriltag_detector_node.log &',
	'roslaunch pi_cam transfer_learning_node.launch > /tmp/transfer_learning_node.log &',
	'roslaunch pi_cam line_detector_node.launch > /tmp/line_detector_node.log &',
	'roslaunch pi_cam face_recognizer_node.launch > /tmp/face_recognizer_node.log &',
	'roslaunch pi_driver joystick_node.launch > /tmp/joystick_node.log &',
		]

launch_cmds2 = [
	'docker run -t -v /home/pi:/home/pi --net host --privileged --rm --name pi_camera wupanhao/lepi_server:melodic bash -c "source env.sh && roslaunch pi_cam camera_node.launch" > /tmp/camera_node.log &',
	'docker run -t -v /home/pi:/home/pi --net host --privileged --rm --name apriltag_detector wupanhao/lepi_server:melodic bash -c "source env.sh && roslaunch pi_cam apriltag_detector_node.launch" > /tmp/apriltag_detector_node.log &',
	'docker run -t -v /home/pi:/home/pi --net host --privileged --rm --name transfer_learning wupanhao/lepi_server:melodic bash -c "source env.sh && roslaunch pi_cam transfer_learning_node.launch" > /tmp/transfer_learning_node.log &',
	'docker run -t -v /home/pi:/home/pi --net host --privileged --rm --name line_detector wupanhao/lepi_server:melodic bash -c "source env.sh && roslaunch pi_cam line_detector_node.launch" > /tmp/line_detector_node.log &',
	'docker run -t -v /home/pi:/home/pi --net host --privileged --rm --name face_recognizer wupanhao/lepi_server:melodic bash -c "source env.sh && roslaunch pi_cam face_recognizer_node.launch" > /tmp/face_recognizer_node.log &',
	'docker run -t -v /home/pi:/home/pi --net host --privileged --rm --name joystick wupanhao/lepi_server:melodic bash -c "source env.sh && roslaunch pi_driver joystick_node.launch" > /tmp/joystick_node.log &',
		]

class PiMasterNode:
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))
		self.srv_get_alive_nodes = rospy.Service('~get_alive_nodes', GetStrings, self.cbGetAliveNodes)
		self.srv_shutdown_node = rospy.Service('~shutdown_node', SetString, self.cbShutdownNode)
		self.srv_launch_node = rospy.Service('~launch_node', SetString, self.cbLaunchNode)
		# self.pub_joy_state =rospy.Publisher("~joy_state", String, queue_size=1)
		rospy.loginfo("[%s] Initialized......" % (self.node_name))
	def cbGetAliveNodes(self,params):
		nodes = rosnode.get_node_names()
		for i in ignores:
			if i in nodes:
				nodes.remove(i)
		return GetStringsResponse(nodes)
	def cbShutdownNode(self,params):
		print(params)
		nodes = rosnode.get_node_names()
		if params.data in nodes:
			rosnode.kill_nodes([params.data])
			time.sleep(0.5)
			return SetStringResponse(u'关闭中')
		else:
			return SetStringResponse(u'节点未启动')
	def cbLaunchNode(self,params):
		print(params)
		nodes = rosnode.get_node_names()
		if params.data in nodes:
			return SetStringResponse(u'节点已启动')
		else:
			return self.launchNode(params.data)

	def launchNode(self,node_name):
		if node_name not in available_nodes:
			return SetStringResponse(u'不支持该节点')
		index = available_nodes.index(node_name)
		try:
			os.system(launch_cmds2[index])
			time.sleep(0.5)
			return SetStringResponse(u'启动中,请稍等')
		except Exception as e:
			print(e)
			return SetStringResponse(u'启动出错')
	def onShutdown(self):
		rospy.loginfo("[%s] shutdown......" % (self.node_name))
if __name__ == '__main__':
	rospy.init_node('pi_master_node', anonymous=False)
	node = PiMasterNode()
	rospy.on_shutdown(node.onShutdown)
	rospy.spin()
