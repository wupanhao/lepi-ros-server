#!/usr/bin/python
#!coding:utf-8

# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py

from pi_driver import Lepi,I2cDriver,ButtonMap,D51Driver
from pi_driver.msg import ButtonEvent,Sensor3Axes,MotorInfo,SensorStatusChange,U8Int32
from pi_driver.srv import SetInt32,GetInt32,SetInt32Response,GetInt32Response,\
		GetMotorsInfo,GetMotorsInfoResponse,SensorGet3Axes,SensorGet3AxesResponse,\
		GetPowerState,GetPowerStateResponse,GetSensorInfo,GetSensorInfoResponse

import rospkg
import rospy

class PiDriverNode:
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))
		self.pub_button_event =rospy.Publisher("~button_event", ButtonEvent, queue_size=1)
		self.pub_sensor_status_change =rospy.Publisher("~sensor_status_change", SensorStatusChange, queue_size=1)
		rospy.Service("~motor_set_type", SetInt32 , self.srvMotorSetType)
		rospy.Service("~motor_get_type", GetInt32 , self.srvMotorGetType)
		rospy.Service("~motor_set_state", SetInt32 , self.srvMotorSetState)
		rospy.Service("~motor_get_state", GetInt32 , self.srvMotorGetState)		
		rospy.Service("~motor_set_enable", SetInt32 , self.srvMotorSetEnable)
		rospy.Service("~motor_get_enable", GetInt32 , self.srvMotorGetEnable)
		rospy.Service("~motor_set_speed", SetInt32 , self.srvMotorSetSpeed)
		rospy.Service("~motor_get_speed", GetInt32 , self.srvMotorGetSpeed)
		rospy.Service("~motor_get_pulse", GetInt32 , self.srvMotorGetPulse)
		rospy.Service('~motor_set_position', SetInt32, self.srvMotorSetPosition)
		rospy.Service('~motor_get_position', GetInt32, self.srvMotorGetPosition)
		rospy.Service('~motors_get_info', GetMotorsInfo, self.srvMotorsGetInfo)
		rospy.Service('~sensor_get_type', GetInt32, self.srvSensorGetType)
		rospy.Service('~sensor_get_value', GetInt32, self.srvSensorGetValue)
		rospy.Service('~sensor_get_info', GetSensorInfo, self.srvSensorGetInfo)
		rospy.Service('~sensor_get_3axes', SensorGet3Axes, self.srvSensorGet3Axes)
		rospy.Service('~9axes_set_enable', SetInt32, self.srvSensor9AxesSetEnable)
		rospy.Service('~get_power_state', GetPowerState, self.srvGetPowerState)
		self.i2c_driver = I2cDriver(self.pubButton)
		self.d51_driver = D51Driver(self.pubSensorChange)
		self.sub_motor_set_pulse = rospy.Subscriber("~motor_set_pulse", U8Int32 , self.cbMotorSetPulse, queue_size=1)
		self.sub_motor_set_speed = rospy.Subscriber("~motor_set_speed", U8Int32 , self.cbMotorSetSpeed, queue_size=1)
		self.sub_motor_set_angle = rospy.Subscriber("~motor_set_angle", U8Int32 , self.cbMotorSetAngle, queue_size=1)
		rospy.loginfo("[%s] Initialized......" % (self.node_name))
	def pubButton(self,btn):
		if ButtonMap.has_key(btn):
			e = ButtonEvent()
			e.value = ButtonMap[btn]
			if 0x81 <= btn and btn <=0x89:
				e.type = 1 # down
			elif 0x01 <= btn and btn <=0x09:
				e.type = 3 # up
			elif 0x11 <= btn and btn <= 0x19:
				e.type = 2 # short
			elif 0x91 <= btn and btn <= 0xa9:
				e.type = 4 # long
			self.pub_button_event.publish(e)
			return True
	def pubSensorChange(self,port,sensor_id,status):
		print(port,sensor_id,status)
		self.pub_sensor_status_change.publish(SensorStatusChange(port,sensor_id,status))
	def cbMotorSetPulse(self,msg):
		Lepi.motor_set_pulse(msg.port,msg.value)
	def cbMotorSetSpeed(self,msg):
		Lepi.motor_set_speed(msg.port,msg.value)
	def cbMotorSetAngle(self,msg):
		Lepi.motor_set_angle(msg.port,msg.value)		
	def srvMotorSetType(self,msg):
		# print(msg)
		Lepi.motor_set_type(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvMotorGetType(self,msg):
		# print(msg)
		value = Lepi.motor_get_type(msg.port)
		return GetInt32Response(value)
	def srvMotorSetState(self,msg):
		# print(msg)
		Lepi.motor_set_state(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvMotorGetState(self,msg):
		# print(msg)
		value = Lepi.motor_get_state(msg.port)
		return GetInt32Response(value)		
	def srvMotorSetEnable(self,msg):
		# print(msg)
		Lepi.motor_set_enable(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvMotorGetEnable(self,msg):
		# print(msg)
		enabled = Lepi.motor_get_enable(msg.port)
		return GetInt32Response(enabled)
	def srvMotorSetSpeed(self,msg):
		# print(msg)
		Lepi.motor_set_speed(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvMotorGetSpeed(self,msg):
		# print(msg)
		speed = Lepi.motor_get_speed(msg.port)
		return GetInt32Response(speed)
	def srvMotorGetPulse(self,msg):
		value = Lepi.motor_get_pulse(msg.port)
		return GetInt32Response(value)		
	def srvMotorSetPosition(self,msg):
		# print(msg)
		Lepi.motor_set_target_position(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvMotorGetPosition(self,msg):
		# print(msg)
		position = Lepi.motor_get_current_position(msg.port)
		return GetInt32Response(position)
	def srvMotorsGetInfo(self,msg):
		# return GetMotorsInfoResponse()
		print(msg)
		infos = GetMotorsInfoResponse()
		# print(MotorInfo())
		for i in [1,2,3,4,5]:
			info = Lepi.motor_get_info(i)
			# print(info)
			infos.motors.append(MotorInfo(info[0],info[1],info[2],info[3]))
		return infos
	def srvSensorGetType(self,msg):
		data = Lepi.sensor_get_type(msg.port)
		return GetInt32Response(data)
	def srvSensorGetValue(self,msg):
		data = Lepi.sensor_get_value(msg.port)
		return GetInt32Response(data)
	def srvSensorGetInfo(self,msg):
		sensor_id = Lepi.sensor_get_type(msg.port)
		value = Lepi.sensor_get_value(msg.port)
		return GetSensorInfoResponse(msg.port,sensor_id,value)
	def srvSensor9AxesSetEnable(self,msg):
		# print(msg)
		if msg.port == 0x46:
			self.i2c_driver.nineAxisSetEnable(msg.value)
			return SetInt32Response(msg.port,msg.value)
		return SetInt32Response(msg.port,msg.value)
	def srvSensorGet3Axes(self,msg):
		# print(msg)
		if msg.id == 1:
			data = self.i2c_driver.readAccData()
		elif msg.id == 2:
			data = self.i2c_driver.readGyroData()
		elif msg.id == 3:
			data = self.i2c_driver.readMagnData()
		else:
			return SensorGet3AxesResponse()
		return SensorGet3AxesResponse(Sensor3Axes(data[0],data[1],data[2]))
	def srvGetPowerState(self,msg):
		data = self.i2c_driver.readBatOcv()
		return GetPowerStateResponse(data[0],data[1],data[3])

if __name__ == '__main__':
	rospy.init_node('pi_driver_node', anonymous=False)
	node = PiDriverNode()
	# print(node.srvMotorsGetInfo(None))
	# rospy.on_shutdown(node.onShutdown)
	# thread.start_new_thread(camera_node.startCaptureRawCV, ())
	rospy.spin()
