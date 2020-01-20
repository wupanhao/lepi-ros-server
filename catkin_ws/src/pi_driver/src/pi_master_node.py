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
from pi_driver.srv import SetInt32,SetInt32Response
from std_msgs.msg import String
import pexpect
import sys

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
		self.process = None
		self.terminal = None
		self.subprocesses = []
		rospy.loginfo("[%s] Initializing......" % (self.node_name))
		rospy.Service('~get_alive_nodes', GetStrings, self.cbGetAliveNodes)
		rospy.Service('~shutdown_node', SetString, self.cbShutdownNode)
		rospy.Service('~launch_node', SetString, self.cbLaunchNode)
		# rospy.Service('~run_process_wait', SetString, self.cbRunProcessWait)
		# rospy.Service('~run_process_background', SetString, self.cbRunProcessBackground)
		rospy.Service('~launch_terminal', SetInt32, self.cbLaunchTerminal)
		# self.pub_joy_state =rospy.Publisher("~joy_state", String, queue_size=1)
		rospy.loginfo("[%s] Initialized......" % (self.node_name))
	def cbGetAliveNodes(self,params):
		nodes = rosnode.get_node_names()
		for i in ignores:
			if i in nodes:
				nodes.remove(i)
		return GetStringsResponse(nodes)
	def cbShutdownNode(self,params):
		rospy.loginfo(params)
		nodes = rosnode.get_node_names()
		if params.data in nodes:
			rosnode.kill_nodes([params.data])
			time.sleep(0.5)
			return SetStringResponse(u'关闭中')
		else:
			return SetStringResponse(u'节点未启动')
	def cbLaunchNode(self,params):
		rospy.loginfo(params)
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
			rospy.loginfo(e)
			return SetStringResponse(u'启动出错')

	def cbRunProcessWait(self,params):
		cmd = params.data
		try:
			process = pexpect.spawn(cmd)
			process.logfile = sys.stdout
			# self.subprocesses.append(process)
			self.process = process
			process.expect(pexpect.EOF)
			# while process.isalive() :
			# 	time.sleep(1)
			return SetStringResponse(process.before)
		except Exception as e:
			rospy.loginfo(e)
			return SetStringResponse(e.message)
	def cbRunProcessBackground(self,params):
		cmd = params.data
		try:
			process = pexpect.spawn(cmd)
			process.logfile = sys.stdout
			# self.subprocesses.append(process)
			self.process = process
			# while process.isalive() :
			# 	time.sleep(1)
			return SetStringResponse('执行中')
		except Exception as e:
			rospy.loginfo(e)
			return SetStringResponse(e.message)
	def cbInputString(self,params):
		data = params.data
		if self.process and self.process.isalive():
			self.process.sendline(data)
			# self.process.send(data)
			return SetStringResponse('输入完成')
		else:
			return SetStringResponse('程序未运行')
	def cbInputChar(self,params):
		data = params.data
		if self.process and self.process.isalive():
			self.process.sendline(data)
			# self.process.send(data)
			return SetStringResponse('输入完成')
		else:
			return SetStringResponse('程序未运行')
	def cbLaunchTerminal(self,params):
		print(params)
		if params.value == 1:
			if self.terminal and self.terminal.isalive():
				terminated=self.terminal.terminate(force=True)
				rospy.loginfo(terminated)
			self.terminal = pexpect.spawn("konsole")
		else:
			if self.terminal and self.terminal.isalive():
				terminated=self.terminal.terminate(force=True)
				rospy.loginfo(terminated)
		return SetInt32Response()
	def onShutdown(self):
		rospy.loginfo("[%s] shutdown......" % (self.node_name))

if __name__ == '__main__':
	rospy.init_node('pi_master_node', anonymous=False)
	node = PiMasterNode()
	nodes = rosnode.get_node_names()
	if '/rosbridge_websocket' not in nodes:
		rospy.loginfo('start lepi_server')
		os.system('docker run -t -v /home/pi:/home/pi --net host --privileged --rm --name lepi_server wupanhao/lepi_server:melodic bash -c "source env.sh && roslaunch pi_driver lepi_server.launch" > /tmp/lepi_server.log &')
	else:
		rospy.loginfo('lepi_server started, ignore')
	rospy.on_shutdown(node.onShutdown)
	rospy.spin()
