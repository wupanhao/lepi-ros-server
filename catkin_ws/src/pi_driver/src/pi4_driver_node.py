#!/usr/bin/python
#!coding:utf-8

# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py

# import rospkg
import time
import rospy
import json
import os
import threading
import RPi.GPIO as GPIO

from pi_driver import I2cDriver, SServo, EEPROM, D51Driver, ButtonListener
from pi_driver.msg import ButtonEvent, Sensor3Axes, MotorInfo, SensorStatusChange, SensorValueChange, U8Int32, ServoInfo,\
    NineAxisValue, NineAxisValueChange
from pi_driver.srv import SetInt32, GetInt32, SetInt32Response, GetInt32Response,\
    GetMotorsInfo, GetMotorsInfoResponse, SensorGet3Axes, SensorGet3AxesResponse,\
    GetPowerState, GetPowerStateResponse, GetSensorInfo, GetSensorInfoResponse
from pi_driver.srv import SetString, SetStringResponse, GetString, GetStringResponse, SetOffset, SetOffsetResponse
from pi_driver.srv import SetServoPosition, SetServoPositionResponse, GetServosInfo, GetServosInfoResponse, SetServoParam, SetServoParamResponse
from pi_driver import vcgencmd
#from pymouse import PyMouse
#from pykeyboard import PyKeyboard

ButtonMap = {
    # type==1
    # button down
    59: 77,  # F1 -> Menu 'M' 77
    103: 38,  # 'ArrowUp': 38
    60: 66,  # F2 -> Back 'B' 66
    105: 37,  # 'ArrowLeft': 37
    28: 13,  # 'Enter': 13
    106: 39,  # 'ArrowRight': 39
    61: 82,  # F3 -> Run 'R' 82
    108: 40,  # 'ArrowDown': 40
    1: 83,  # Esc -> Stop 'S' 83
}


class PiDriverNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.is_shutdown = False
        #self.mouse = PyMouse()
        #self.keyboard = PyKeyboard()
        self.update_frequence = 0
        self.nine_axis_update_frequence = 0
        self.frame_count = 0

        self.fan_pin = 14  # 定义pwm输出引脚
        GPIO.setmode(GPIO.BCM)  # 定义树莓派gpio引脚以BCM方式编号
        GPIO.setup(self.fan_pin, GPIO.OUT)  # 使能gpio口为输出
        # self.pwm = GPIO.PWM(pwm_pin,25000)   #定义pwm输出频率

        self.pub_button_event = rospy.Publisher(
            "~button_event", ButtonEvent, queue_size=1)
        self.pub_sensor_status_change = rospy.Publisher(
            "~sensor_status_change", SensorStatusChange, queue_size=1)
        self.pub_sensor_value_change = rospy.Publisher(
            "~sensor_value_change", SensorValueChange, queue_size=1)
        self.pub_nine_axis_value_change = rospy.Publisher(
            "~nine_axis_value_change", NineAxisValueChange, queue_size=1)
        self.d51_driver = D51Driver(onSensorChange=self.pubSensorChange)
        rospy.Service("~motor_set_type", SetInt32, self.srvMotorSetType)
        rospy.Service("~motor_get_type", GetInt32, self.srvMotorGetType)
        # rospy.Service("~motor_set_state", SetInt32, self.srvMotorSetState)
        # rospy.Service("~motor_get_state", GetInt32, self.srvMotorGetState)
        # rospy.Service("~motor_set_enable", SetInt32, self.srvMotorSetEnable)
        # rospy.Service("~motor_get_enable", GetInt32, self.srvMotorGetEnable)
        rospy.Service("~motor_set_speed", SetInt32, self.srvMotorSetSpeed)
        rospy.Service("~motor_set_rotate", SetInt32, self.srvMotorSetRotate)
        rospy.Service("~motor_get_speed", GetInt32, self.srvMotorGetSpeed)
        rospy.Service("~motor_get_pulse", GetInt32, self.srvMotorGetPulse)
        rospy.Service('~motor_set_current_position', SetInt32,
                      self.srvMotorSetCurrentPosition)
        rospy.Service('~motor_set_position', SetInt32,
                      self.srvMotorSetPosition)
        rospy.Service('~motor_get_position', GetInt32,
                      self.srvMotorGetPosition)
        rospy.Service('~motors_get_info', GetMotorsInfo, self.srvMotorsGetInfo)
        rospy.Service('~sensor_get_type', GetInt32, self.srvSensorGetType)
        rospy.Service('~sensor_get_mode', GetInt32, self.srvSensorGetMode)
        rospy.Service('~sensor_set_mode', SetInt32, self.srvSensorSetMode)
        rospy.Service('~sensor_get_value', GetInt32, self.srvSensorGetValue)
        rospy.Service('~sensor_set_value', SetInt32, self.srvSensorSetValue)
        # rospy.Service('~sensor_get_info', GetSensorInfo, self.srvSensorGetInfo)
        rospy.Service('~sensors_get_info', GetMotorsInfo,
                      self.srvSensorsGetInfo)
        rospy.Service('~sensor_get_3axes', SensorGet3Axes,
                      self.srvSensorGet3Axes)
        rospy.Service('~sensor_get_offset_3axes', SensorGet3Axes,
                      self.srvSensorGetOffset3Axes)
        rospy.Service('~sensor_get_offset', SensorGet3Axes,
                      self.srvSensorGetOffset)
        rospy.Service('~sensor_set_offset', SetOffset,
                      self.srvSensorSetOffset)
        rospy.Service('~sensor_estimate_pose', SensorGet3Axes,
                      self.srvEstimatePose)
        rospy.Service('~9axes_set_enable', SetInt32,
                      self.srvSensor9AxesSetEnable)
        rospy.Service('~get_power_state', GetPowerState, self.srvGetPowerState)
        rospy.Service('~system_poweroff', SetInt32,
                      self.srvSystemPoweroff)
        rospy.Service('~system_get_power_meas', SensorGet3Axes,
                      self.srvSystemGetPowerMeas)
        rospy.Service('~system_get_vout1', SensorGet3Axes,
                      self.srvSystemGetVout1)
        rospy.Service('~system_get_vout2', SensorGet3Axes,
                      self.srvSystemGetVout2)
        # rospy.Service('~input_string', SetString, self.srvInputString)
        # rospy.Service('~input_char', SetInt32, self.srvInputChar)
        # rospy.Service('~mouse_click', SetString, self.cbMouseClick)
        rospy.Service('~variable_list', GetString, self.srvGetVariableList)
        rospy.Service('~servo_get_u8', SetServoParam, self.srvServoGetU8)
        rospy.Service('~servo_get_u16', SetServoParam, self.srvServoGetU16)
        rospy.Service('~servo_set_u8', SetServoParam, self.srvServoSetU8)
        rospy.Service('~servo_set_u16', SetServoParam, self.srvServoSetU16)
        rospy.Service('~servo_set_position', SetServoPosition,
                      self.srvServoSetPosition)
        rospy.Service('~servos_get_info', GetServosInfo, self.srvServosGetInfo)
        rospy.Service('~get_firmware_version', GetInt32,
                      self.srvGetFirmwareVersion)
        self.i2c_driver = I2cDriver()
        self.servo = SServo('/dev/ttyAMA1')
        self.btnListener = ButtonListener(self.pubButton)
        self.sub_motor_set_pulse = rospy.Subscriber(
            "~motor_set_pulse", U8Int32, self.cbMotorSetPulse, queue_size=1)
        self.sub_motor_set_speed = rospy.Subscriber(
            "~motor_set_speed", U8Int32, self.cbMotorSetSpeed, queue_size=1)
        self.sub_motor_set_angle = rospy.Subscriber(
            "~motor_set_angle", U8Int32, self.cbMotorSetAngle, queue_size=1)
        rospy.Service('~set_update_frequence', SetInt32,
                      self.srvSetUpdateFrequence)
        reader = threading.Thread(target=self.start_update_loop)
        # reader.daemon = True
        reader.start()

        rospy.Service('~set_nine_axis_update_frequence', SetInt32,
                      self.srvSetNineAxisUpdateFrequence)
        reader2 = threading.Thread(target=self.start_nine_axis_update_loop)
        # reader.daemon = True
        reader2.start()

        rospy.Timer(rospy.Duration.from_sec(3.0), self.frame_counter)
        rospy.loginfo("[%s] Initialized......" % (self.node_name))

    def frame_counter(self, channel=None):
        print(self.d51_driver.frame_count - self.frame_count, 'frames')
        self.frame_count = self.d51_driver.frame_count
        temp = vcgencmd.measure_temp()
        if temp > 70:
            GPIO.output(self.fan_pin, 1)
            # self.pwm.start(100)
        elif temp < 65:
            GPIO.output(self.fan_pin, 0)
            # self.pwm.start(0)

    def pubButton(self, event):
        if event.code in ButtonMap:
            e = ButtonEvent()
            e.value = ButtonMap[event.code]
            if event.value == 1:
                e.type = 1  # down
            elif event.value == 0:
                e.type = 3  # up
            elif event.value == 0:
                e.type = 2  # short
            elif event.value == 2:
                e.type = 4  # long,hold
            self.pub_button_event.publish(e)
            return True

    def pubSensorChange(self, port, sensor_id, status):
        print(port, sensor_id, status)
        self.pub_sensor_status_change.publish(
            SensorStatusChange(port, sensor_id, status))

    def cbMotorSetPulse(self, params):
        self.d51_driver.motor_set_pulse(params.port, params.value)

    def cbMotorSetSpeed(self, params):
        self.d51_driver.motor_set_speed(params.port, params.value)

    def cbMotorSetAngle(self, params):
        self.d51_driver.motor_set_angle(params.port, params.value)

    def srvMotorSetType(self, params):
        self.d51_driver.motor_set_type(params.port, params.value)
        if params.value == 1:
            self.d51_driver._motor_set_pulse(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetType(self, params):
        # print(params)
        value = self.d51_driver.motor_get_type(params.port)
        return GetInt32Response(value)

    # def srvMotorSetState(self, params):
    #     return SetInt32Response(params.port, params.value)

    # def srvMotorGetState(self, params):
    #     return GetInt32Response()

    # def srvMotorSetEnable(self, params):
    #     return SetInt32Response(params.port, params.value)

    # def srvMotorGetEnable(self, params):
    #     return GetInt32Response()

    def srvMotorSetSpeed(self, params):
        self.d51_driver.motor_set_speed(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetSpeed(self, params):
        value = self.d51_driver.motor_get_speed(params.port)
        return GetInt32Response(value)

    def srvMotorSetRotate(self, params):
        position = self.d51_driver.motor_get_position(params.port)
        self.d51_driver.motor_set_point(params.port, params.value*2+position)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetPulse(self, params):
        value = self.d51_driver.motor_get_pulse(params.port)
        return GetInt32Response()

    def srvMotorSetPosition(self, params):
        self.d51_driver.motor_set_point(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorSetCurrentPosition(self, params):
        if params.port == 0:
            for i in range(5):
                self.d51_driver.motor_set_position(i+1, params.value)
        else:
            self.d51_driver.motor_set_position(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    def srvMotorGetPosition(self, params):
        value = self.d51_driver.motor_get_position(params.port)
        return GetInt32Response(value)

    def srvMotorsGetInfo(self, params):
        # return GetMotorsInfoResponse()
        # print(params)
        infos = GetMotorsInfoResponse()
        # print(MotorInfo())
        for i in [1, 2, 3, 4, 5]:
            info = self.d51_driver.motor_get_info(i)
            # print(info)
            infos.motors.append(MotorInfo(info[0], info[1], info[2], info[3]))
        return infos

    def srvSensorsGetInfo(self, params):
        # return GetMotorsInfoResponse()
        # print(params)
        infos = GetMotorsInfoResponse()
        # print(MotorInfo())
        for i in [1, 2, 3, 4, 5]:
            info = self.d51_driver.sensor_get_info(i)
            # print(info)
            infos.motors.append(MotorInfo(info[0], info[1], info[2], info[3]))
        return infos

    def srvSensorGetType(self, params):
        value = self.d51_driver.sensor_get_type(params.port)
        return GetInt32Response(value)

    def srvSensorGetValue(self, params):
        # print(params)
        value = self.d51_driver.sensor_get_value(params.port)
        return GetInt32Response(value)

    def srvSensorSetValue(self, params):
        self.d51_driver.sensor_set_value(params.port, params.value)
        return SetInt32Response(params.port, params.value)

    # def srvSensorGetInfo(self, params):
    #     return GetSensorInfoResponse(params.port, 0, 0)

    def srvSensorGetMode(self, params):
        value = self.d51_driver.sensor_get_mode(params.port)
        return GetInt32Response(value)

    def srvSensorSetMode(self, params):
        self.d51_driver.sensor_set_mode(params.port, params.value)
        return SetInt32Response(params.port, params.value)

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

    def srvSensorGetOffset3Axes(self, params):
        # print(params)
        if params.id == 1:
            data = self.i2c_driver.readAccData(True)
        elif params.id == 2:
            data = self.i2c_driver.readGyroData(True)
        elif params.id == 3:
            data = self.i2c_driver.readMagnData(True)
        else:
            return SensorGet3AxesResponse()
        return SensorGet3AxesResponse(Sensor3Axes(data[0], data[1], data[2]))

    def srvSensorGetOffset(self, params):
        # print(params)
        if params.id == 1:
            data = self.i2c_driver.offset['acc']
        elif params.id == 2:
            data = self.i2c_driver.offset['gyro']
        elif params.id == 3:
            data = self.i2c_driver.offset['magn']
        else:
            return SensorGet3AxesResponse()
        return SensorGet3AxesResponse(Sensor3Axes(data[0], data[1], data[2]))

    def srvSensorSetOffset(self, params):
        # print(params)
        data = params.data
        data = [data.x, data.y, data.z]
        if params.id == 1:
            self.i2c_driver.offset['acc'] = data
        elif params.id == 2:
            self.i2c_driver.offset['gyro'] = data
        elif params.id == 3:
            self.i2c_driver.offset['magn'] = data
        else:
            return SetOffsetResponse()
        return SetOffsetResponse(Sensor3Axes(data[0], data[1], data[2]))

    def srvEstimatePose(self, params):
        data = self.i2c_driver.estimatePose()
        return SensorGet3AxesResponse(Sensor3Axes(data[0], data[1], data[2]))

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
            ids = self.servo.scan()
        else:
            ids = params.ids
        print(ids)
        for servo_id in ids:
            info = ServoInfo()
            info.id = servo_id
            # print(servo_id)
            info.min_position = self.servo.read_u16(
                servo_id, EEPROM.MIN_POSITION_H)
            info.cur_position = self.servo.read_u16(
                servo_id, EEPROM.CURRENT_POSITION_H)
            info.max_position = self.servo.read_u16(
                servo_id, EEPROM.MAX_POSITION_H)
            rsp.servos.append(info)
        return rsp

    def srvServoSetPosition(self, params):
        status = self.servo.set_position(
            params.id, params.position, params.ms, params.speed)
        return SetServoPositionResponse(status)

    def srvServoSetU8(self, params):
        if params.param_id == 0x99:
            status = self.servo.reset(params.id)
        else:
            status = self.servo.write_u8(
                params.id, params.param_id, params.value)
        return SetServoParamResponse(status)

    def srvServoSetU16(self, params):
        status = self.servo.write_u16(params.id, params.param_id, params.value)
        return SetServoParamResponse(status)

    def srvServoGetU8(self, params):
        status = self.servo.read_u8(params.id, params.param_id)
        return SetServoParamResponse(status)

    def srvServoGetU16(self, params):
        status = self.servo.read_u16(params.id, params.param_id)
        return SetServoParamResponse(status)

    def srvGetFirmwareVersion(self, params):
        self.d51_driver._system_get_version()
        return GetInt32Response(self.d51_driver.system.version)

    def srvGetPowerState(self, params):
        # charging bat_power_ocv est_power
        # self.d51_driver._system_get_power()
        data = [self.d51_driver.system.charge_state,
                4.1, self.d51_driver.system.battery_level]
        return GetPowerStateResponse(data[0], data[1], data[2])

    def srvSystemPoweroff(self, params):
        self.d51_driver.system_poweroff()
        os.system('bash -c "sleep 2 && sudo halt" &')
        return SetInt32Response()

    def srvSystemGetPowerMeas(self, params):
        data = self.d51_driver.system_get_power_meas()
        return SensorGet3AxesResponse(Sensor3Axes(data[0], data[1], data[2]))

    def srvSystemGetVout1(self, params):
        data = self.d51_driver.system_get_vout1()
        return SensorGet3AxesResponse(Sensor3Axes(data[0], data[1], data[2]))

    def srvSystemGetVout2(self, params):
        data = self.d51_driver.system_get_vout2()
        return SensorGet3AxesResponse(Sensor3Axes(data[0], data[1], data[2]))

    def onShutdown(self):
        self.is_shutdown = True
        self.d51_driver.active = False
        rospy.loginfo("[%s] shutdown......" % (self.node_name))

    def srvSetUpdateFrequence(self, params):
        print(params)
        if params.value < 0 or params.value > 30:
            return SetInt32Response()
        else:
            self.update_frequence = params.value
            return SetInt32Response(0, params.value)

    def start_update_loop(self):
        # print(self.d51_driver.value_updated)
        while not self.is_shutdown:
            if self.update_frequence == 0:
                time.sleep(1)
                continue
            msg = SensorValueChange()
            if sum(self.d51_driver.value_updated.values()) > 0:
                for (k, v) in self.d51_driver.value_updated.items():
                    if v and k <= 5:
                        self.d51_driver.value_updated[k] = False
                        msg.data.append(
                            U8Int32(k, self.d51_driver.sensor_get_value(k)))
                    elif v and k <= 10:
                        self.d51_driver.value_updated[k] = False
                        msg.data.append(
                            U8Int32(k, self.d51_driver.motor_get_position(k-5)))
                self.pub_sensor_value_change.publish(msg)
            time.sleep(1.0/self.update_frequence)

    def srvSetNineAxisUpdateFrequence(self, params):
        print(params)
        if params.value < 0 or params.value > 100:
            return SetInt32Response()
        else:
            self.nine_axis_update_frequence = params.value
            return SetInt32Response(0, params.value)

    def start_nine_axis_update_loop(self):
        # print(self.d51_driver.value_updated)
        while not self.is_shutdown:
            if self.nine_axis_update_frequence == 0:
                time.sleep(1)
                continue
            msg = NineAxisValueChange()
            data = [self.i2c_driver.readAccData(True), self.i2c_driver.readGyroData(
                True), self.i2c_driver.readMagnData(True)]
            for i in range(3):
                msg.data.append(NineAxisValue(
                    i+1, data[i][0], data[i][1], data[i][2]))
            self.pub_nine_axis_value_change.publish(msg)
            time.sleep(1.0/self.nine_axis_update_frequence)


if __name__ == '__main__':
    rospy.init_node('pi_driver_node', anonymous=False)
    node = PiDriverNode()
    # print(node.srvMotorsGetInfo(None))
    rospy.on_shutdown(node.onShutdown)
    # thread.start_new_thread(camera_node.startCaptureRawCV, ())
    rospy.spin()
