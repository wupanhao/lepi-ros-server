#!/usr/bin/python
#!coding:utf-8

# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py

import rospkg
import rospy
import json

from pi_driver import Lepi, I2cDriver, ButtonMap, D51Driver, EEPROM
from pi_driver.msg import ButtonEvent, Sensor3Axes, MotorInfo, SensorStatusChange, U8Int32, ServoInfo
from pi_driver.srv import SetInt32, GetInt32, SetInt32Response, GetInt32Response,\
    GetMotorsInfo, GetMotorsInfoResponse, SensorGet3Axes, SensorGet3AxesResponse,\
    GetPowerState, GetPowerStateResponse, GetSensorInfo, GetSensorInfoResponse
from pi_driver.srv import SetString, SetStringResponse, GetString, GetStringResponse
from pi_driver.srv import SetServoPosition , SetServoPositionResponse, GetServosInfo, GetServosInfoResponse,SetServoParam,SetServoParamResponse

#from pymouse import PyMouse
#from pykeyboard import PyKeyboard


class PiDriverNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        #self.mouse = PyMouse()
        #self.keyboard = PyKeyboard()
        self.pub_button_event = rospy.Publisher(
            "~button_event", ButtonEvent, queue_size=1)
        self.pub_sensor_status_change = rospy.Publisher(
            "~sensor_status_change", SensorStatusChange, queue_size=1)
        rospy.Service("~motor_set_type", SetInt32, self.srvMotorSetType)
        rospy.Service("~motor_get_type", GetInt32, self.srvMotorGetType)
        rospy.Service("~motor_set_state", SetInt32, self.srvMotorSetState)
        rospy.Service("~motor_get_state", GetInt32, self.srvMotorGetState)
        rospy.Service("~motor_set_enable", SetInt32, self.srvMotorSetEnable)
        rospy.Service("~motor_get_enable", GetInt32, self.srvMotorGetEnable)
        rospy.Service("~motor_set_speed", SetInt32, self.srvMotorSetSpeed)
        rospy.Service("~motor_get_speed", GetInt32, self.srvMotorGetSpeed)
        rospy.Service("~motor_get_pulse", GetInt32, self.srvMotorGetPulse)
        rospy.Service('~motor_set_position', SetInt32,
                      self.srvMotorSetPosition)
        rospy.Service('~motor_get_position', GetInt32,
                      self.srvMotorGetPosition)
        rospy.Service('~motors_get_info', GetMotorsInfo, self.srvMotorsGetInfo)
        rospy.Service('~sensor_get_type', GetInt32, self.srvSensorGetType)
        rospy.Service('~sensor_get_value', GetInt32, self.srvSensorGetValue)
        rospy.Service('~sensor_get_info', GetSensorInfo, self.srvSensorGetInfo)
        rospy.Service('~sensor_get_3axes', SensorGet3Axes,
                      self.srvSensorGet3Axes)
        rospy.Service('~9axes_set_enable', SetInt32,
                      self.srvSensor9AxesSetEnable)
        rospy.Service('~get_power_state', GetPowerState, self.srvGetPowerState)
        # rospy.Service('~input_string', SetString, self.srvInputString)
        # rospy.Service('~input_char', SetInt32, self.srvInputChar)
        # rospy.Service('~mouse_click', SetString, self.cbMouseClick)
        rospy.Service('~variable_list', GetString, self.srvGetVariableList)
        rospy.Service('~servo_get_u8', SetServoParam, self.srvServoGetU8)
        rospy.Service('~servo_get_u16', SetServoParam, self.srvServoGetU16)
        rospy.Service('~servo_set_u8', SetServoParam, self.srvServoSetU8)
        rospy.Service('~servo_set_u16', SetServoParam, self.srvServoSetU16)
        rospy.Service('~servo_set_position', SetServoPosition, self.srvServoSetPosition)
        rospy.Service('~servos_get_info', GetServosInfo, self.srvServosGetInfo)
        self.i2c_driver = I2cDriver(self.pubButton)
        self.d51_driver = D51Driver(self.pubSensorChange)
        self.sub_motor_set_pulse = rospy.Subscriber(
            "~motor_set_pulse", U8Int32, self.cbMotorSetPulse, queue_size=1)
        self.sub_motor_set_speed = rospy.Subscriber(
            "~motor_set_speed", U8Int32, self.cbMotorSetSpeed, queue_size=1)
        self.sub_motor_set_angle = rospy.Subscriber(
            "~motor_set_angle", U8Int32, self.cbMotorSetAngle, queue_size=1)
        rospy.loginfo("[%s] Initialized......" % (self.node_name))

    def pubButton(self, btn):
        if ButtonMap.has_key(btn):
            e = ButtonEvent()
            e.value = ButtonMap[btn]
            if 0x81 <= btn and btn <= 0x89:
                e.type = 1  # down
            elif 0x01 <= btn and btn <= 0x09:
                e.type = 3  # up
            elif 0x11 <= btn and btn <= 0x19:
                e.type = 2  # short
            elif 0x91 <= btn and btn <= 0xa9:
                e.type = 4  # long
            self.pub_button_event.publish(e)
            return True

    def pubSensorChange(self, port, sensor_id, status):
        print(port, sensor_id, status)
        self.pub_sensor_status_change.publish(
            SensorStatusChange(port, sensor_id, status))

    def cbMotorSetPulse(self, msg):
        Lepi.motor_set_pulse(msg.port, msg.value)

    def cbMotorSetSpeed(self, msg):
        Lepi.motor_set_speed(msg.port, msg.value)

    def cbMotorSetAngle(self, msg):
        Lepi.motor_set_angle(msg.port, msg.value)

    def srvMotorSetType(self, params):
        # print(params)
        Lepi.motor_set_type(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetType(self, params):
        # print(params)
        value = Lepi.motor_get_type(params.port)
        return GetInt32Response(value)

    def srvMotorSetState(self, params):
        # print(params)
        Lepi.motor_set_state(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetState(self, params):
        # print(params)
        value = Lepi.motor_get_state(params.port)
        return GetInt32Response(value)

    def srvMotorSetEnable(self, params):
        # print(params)
        Lepi.motor_set_enable(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetEnable(self, params):
        # print(params)
        enabled = Lepi.motor_get_enable(params.port)
        return GetInt32Response(enabled)

    def srvMotorSetSpeed(self, params):
        # print(params)
        Lepi.motor_set_speed(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetSpeed(self, params):
        # print(params)
        speed = Lepi.motor_get_speed(params.port)
        return GetInt32Response(speed)

    def srvMotorGetPulse(self, params):
        value = Lepi.motor_get_pulse(params.port)
        return GetInt32Response(value)

    def srvMotorSetPosition(self, params):
        # print(params)
        Lepi.motor_set_target_position(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetPosition(self, params):
        # print(params)
        position = Lepi.motor_get_current_position(params.port)
        return GetInt32Response(position)

    def srvMotorsGetInfo(self, params):
        # return GetMotorsInfoResponse()
        print(params)
        infos = GetMotorsInfoResponse()
        # print(MotorInfo())
        for i in [1, 2, 3, 4, 5]:
            info = Lepi.motor_get_info(i)
            # print(info)
            infos.motors.append(MotorInfo(info[0], info[1], info[2], info[3]))
        return infos

    def srvSensorGetType(self, params):
        data = Lepi.sensor_get_type(params.port)
        return GetInt32Response(data)

    def srvSensorGetValue(self, params):
        data = Lepi.sensor_get_value(params.port)
        return GetInt32Response(data)

    def srvSensorGetInfo(self, params):
        sensor_id = Lepi.sensor_get_type(params.port)
        value = Lepi.sensor_get_value(params.port)
        return GetSensorInfoResponse(params.port, sensor_id, value)

    def srvSensor9AxesSetEnable(self, params):
        # print(params)
        if params.port == 0x46:
            self.i2c_driver.nineAxisSetEnable(params.value)
            return SetInt32Response(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvSensorGet3Axes(self, params):
        # print(params)
        if params.id == 1:
            data = self.i2c_driver.readAccData()
        elif params.id == 2:
            data = self.i2c_driver.readGyroData()
        elif params.id == 3:
            data = self.i2c_driver.readMagnData()
        else:
            return SensorGet3AxesResponse()
        return SensorGet3AxesResponse(Sensor3Axes(data[0], data[1], data[2]))

    def srvGetPowerState(self, params):
        data = self.i2c_driver.readBatOcv()
        return GetPowerStateResponse(data[0], data[1], data[3])
    '''
    def srvInputString(self, params):
        try:
            self.mouse.click(2, 2, 1)
            self.keyboard.type_string(params.data)
            self.keyboard.tap_key(self.keyboard.enter_key)
            return SetStringResponse('输入完成')
        except Exception as e:
            rospy.loginfo(e)
            return SetStringResponse('输入出错:'+e.message)

    def srvInputChar(self, params):
        if params.port == 1:
            self.keyboard.press_key(params.value)
        elif params.port == 2:
            self.keyboard.tap_key(params.value)
        elif params.port == 3:
            self.keyboard.release_key(params.value)
        return SetInt32Response()
    '''
    def srvGetVariableList(self, params):
        variable_list = {}
        try:
            variable_list = rospy.get_param('/variable')
        except Exception as e:
            print(e)
        return GetStringResponse(json.dumps(variable_list))

    def srvServosGetInfo(self, params):
        # print(params)
        rsp = GetServosInfoResponse()
        if len(params.ids) == 0:
            ids = Lepi.servo_scan()
        else:
            ids = params.ids
        print(ids)
        for servo_id in ids:
            info = ServoInfo()
            info.id = servo_id
            # print(servo_id)
            info.min_position = Lepi.servo_read_u16(servo_id,EEPROM.MIN_POSITION_H)
            info.cur_position = Lepi.servo_read_u16(servo_id,EEPROM.CURRENT_POSITION_H)
            info.max_position = Lepi.servo_read_u16(servo_id,EEPROM.MAX_POSITION_H)
            rsp.servos.append(info)
        return rsp

    def srvServoSetPosition(self,params):
        status = Lepi.servo_set_position(params.id,params.position,params.ms,params.speed)
        return SetServoPositionResponse(status)

    def srvServoSetU8(self,params):
        if params.param_id == 0x99:
            status = Lepi.servo_reset(params.id)
        else:
            status = Lepi.servo_write_u8(params.id,params.param_id,params.value)
        return SetServoParamResponse(status)

    def srvServoSetU16(self,params):
        status = Lepi.servo_write_u16(params.id,params.param_id,params.value)
        return SetServoParamResponse(status)

    def srvServoGetU8(self,params):
        status = Lepi.servo_read_u8(params.id,params.param_id)
        return SetServoParamResponse(status)

    def srvServoGetU16(self,params):
        status = Lepi.servo_read_u16(params.id,params.param_id)
        return SetServoParamResponse(status)

if __name__ == '__main__':
    rospy.init_node('pi_driver_node', anonymous=False)
    node = PiDriverNode()
    # print(node.srvMotorsGetInfo(None))
    # rospy.on_shutdown(node.onShutdown)
    # thread.start_new_thread(camera_node.startCaptureRawCV, ())
    rospy.spin()
