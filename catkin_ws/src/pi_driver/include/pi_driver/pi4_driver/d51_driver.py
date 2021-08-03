#!coding:utf-8
import serial
import time
import ctypes
import threading
from collections import deque

header = [0x55, 0xaa]


def chk_sum(data):
    # data[-1] = 0
    # print(data)
    check = sum(data[2:]) & 0xFF
    return check


def to_int32(value):
    cint32 = ctypes.c_int32((value[3] << 24) | (
        value[2] << 16) | (value[1] << 8) | value[0])
    return cint32.value


def to_value(data):
    l = len(data)
    if l == 1:
        return data[0]
    elif l == 2:
        return data[0] | (data[1] << 8)
    elif l >= 4 and l%4==0:
        n = l/4
        arr = []
        for i in range(n):
            arr.append(data[i*4:i*4+4])
        print(arr)
        return to_int32(data[:4])
    else:
        print(data)
        return 0


def remap_sensor_id(id):
    return 6 - id


class System:
    def __init__(self):
        self.version = 0
        self.battery_level = 50
        self.charge_state = 0
        self.battery_mV = 0
        self.battery_mA = 0
        self.vout1_mV = 0
        self.vout1_mA = 0
        self.vout2_mV = 0
        self.vout2_mA = 0

    def __repr__(self):
        return str({"version": self.version, "battery_level": self.battery_level,
                    "charge_state": self.charge_state, "battery_mV": self.battery_mV, "battery_mA": self.battery_mA, })


class Motor:
    def __init__(self, port):
        self.port = port
        self.type = 0
        self.pulse = 0
        self.position = 0
        self.setpoint = 0

    def __repr__(self):
        return str({"port": self.port, "type": self.type, "pulse": self.pulse,
                    "position": self.position, "setpoint": self.setpoint})


class Sensor:
    def __init__(self, port):
        self.port = port
        self.type = 0
        self.mode = 0
        self.value = 0
        self.data = []
        self.raw_data = []

    def __repr__(self):
        return str({"port": self.port, "type": self.type, "mode": self.mode, "value": self.value,
                    "data": self.data, "raw_data": self.raw_data})


class D51Driver(object):
    """
    serial client
    """

    def __init__(self, port='/dev/ttyACM0', baud_rate=115200, onSensorChange=None, debug_mode=False):
        self.port = serial.Serial(port=port, baudrate=baud_rate, bytesize=8, parity=serial.PARITY_NONE,
                                  #   timeout=0.001,
                                  stopbits=serial.STOPBITS_ONE, dsrdtr=False)
        self.active = True
        self.onSensorChange = onSensorChange
        if onSensorChange is not None:
            threading._start_new_thread(self.start_listen_loop2, ())
        self.debug_mode = debug_mode
        self.system = System()
        self._system_get_version()
        self._system_get_power()
        self._system_get_charge()
        self.system_get_power_meas()
        self.motor = {}
        self.sensor = {}
        self.frame_count = 0
        # 1-5 for sensor data 6-10 for motor position
        self.value_updated = {
            1: False,
            2: False,
            3: False,
            4: False,
            5: False,
            6: False,
            7: False,
            8: False,
            9: False,
            10: False,
        }
        for i in range(5):
            self.motor[i+1] = Motor(i+1)
            self.sensor[i+1] = Sensor(i+1)
            self._motor_get_type(i+1)
            self._motor_get_pulse(i+1)
            self._motor_get_position(i+1)
            self._sensor_get_type(i+1)
            self._sensor_get_mode(i+1)
            self._sensor_get_value(i+1)

    def send_str(self, cmd):
        self.port.write(cmd.encode('utf-8'))

    def read_str(self):
        response = self.port.readline()
        return response.decode('utf-8')

    def read_num(self, num):
        response = self.port.read(num)
        return response

    def send_hex(self, cmd):
        # print(bytes(cmd))
        self.port.write(cmd)
        # time.sleep(0.00001)
        # self.port.flushInput()

    def read_hex(self, n=0):
        if(n > 0):
            response = self.port.read(n)
        else:
            response = self.port.readline()
        array = []
        for i in response:
            array.append(ord(i))
        return array[0] if len(array) == 1 else array

    def write_32(self, id, value):
        data = [header[0], header[1], id, 0x04, (value & 0xFF), ((value >> 8) & 0xFF), ((
            value >> 16) & 0xFF), ((value >> 24) & 0xFF)]
        data.append(chk_sum(data))
        self.send_hex(data)
        # time.sleep(0.001)

    def read_32(self, id):
        data = [header[0], header[1], id, 0x00]
        data.append(chk_sum(data))
        self.send_hex(data)
        # time.sleep(0.001)
        # res = self.read_hex()
        # self.on_data(res)
        # print(res)

    def start_listen_loop(self):
        # 需要设置timeout
        while True:
            try:
                data = self.read_hex()
                if len(data) == 0:
                    time.sleep(0.002)
                else:
                    self.on_data(data)
            except Exception as e:
                print(e)
                time.sleep(0.5)

    def start_listen_loop2(self):
        data = deque([0x00, 0x00])
        while self.active:
            try:
                data.popleft()
                data.append(self.read_hex(1))
                if data[0] == header[0] and data[1] == header[1]:
                    attr_id, length = self.read_hex(2)
                    value = self.read_hex(length+1)
                    # print(attr_id, value, length)
                    self.on_frame(attr_id, value[:-1])
                # print(data)
                # elif header[0] not in data:
                #     print('bad data', data)
            except Exception as e:
                print(e)
                time.sleep(0.5)

    def on_data(self, data):
        l = len(data)
        if l < 6:
            return
        if data[0] == 0x55 and data[1] == 0xaa:
            if l >= data[3]+5:
                frame = data[:data[3]+5]
                attr_id = frame[2]
                length = frame[3]
                value = frame[4:4+length]
                self.on_frame(attr_id, value)
                if l >= data[3]+10:
                    self.on_data(data[data[3]+5:])
        elif 0x55 in data[1:]:
            print('bad frame', data)
            start = data[1:].index(0x55)
            self.on_data(data[1+start:])
        else:
            print('bad data', data)
            pass
        # print(self.motor)

    def on_frame(self, attr_id, value):
        self.frame_count = self.frame_count + 1
        # print(attr_id, length, value)
        if attr_id < 0x10:
            self.on_system(attr_id, value)
        if attr_id >= 0x10 and attr_id < 0x60:
            self.on_motor(attr_id, value)
        if attr_id >= 0x60 and attr_id < 0xb0:
            self.on_sensor(attr_id, value)

    def on_system(self, attr_id, data):
        attr = attr_id
        value = to_int32(data)
        if attr == 0:
            self.system.version = value
        elif attr == 1:
            self.system.battery_level = value
        elif attr == 2:
            self.system.charge_state = value & 0x01
        elif attr == 4:
            self.system.battery_mA = value & 0xFFFF
            self.system.battery_mV = value >> 16
        elif attr == 5:
            self.system.vout1_mA = value & 0xFFFF
            self.system.vout1_mV = value >> 16
        elif attr == 6:
            self.system.vout2_mA = value & 0xFFFF
            self.system.vout2_mV = value >> 16
        print(self.system)

    def on_motor(self, attr_id, data):
        motor_id = attr_id >> 4
        attr = attr_id & 0x0F
        value = to_int32(data)
        if attr == 0:
            self.motor[motor_id].type = value
        elif attr == 1:
            self.motor[motor_id].pulse = value
        elif attr == 2:
            self.motor[motor_id].position = value
        elif attr == 3:
            self.motor[motor_id].setpoint = value
        if self.debug_mode:
            print(self.motor[motor_id])
        if motor_id <= 5 and attr == 2:
            self.value_updated[motor_id+5] = True

    def on_sensor(self, attr_id, data):
        sensor_id = (attr_id >> 4)-5
        sensor_id = remap_sensor_id(sensor_id)
        attr = attr_id & 0x0F
        value = to_int32(data[-4:])
        if attr == 0:
            if self.sensor[sensor_id].type != value:
                self.sensor[sensor_id].type = value
                if self.onSensorChange is not None:
                    self.onSensorChange(
                        sensor_id, value, int(value != 0)*2 - 1)
            print(self.sensor[sensor_id])
        elif attr == 1:
            self.sensor[sensor_id].mode = value
            print(self.sensor[sensor_id])
        elif attr == 2:
            self.sensor[sensor_id].raw_data = data
            self.sensor[sensor_id].data = self.on_sensor_data(data)
            self.sensor[sensor_id].mode = data[0] & 0x07
            self.sensor[sensor_id].value = to_value(
                self.sensor[sensor_id].data)
            # print(self.sensor[sensor_id].value)
        if sensor_id <= 5 and attr == 2:
            self.value_updated[sensor_id] = True

    def on_sensor_data(self, data):
        mode = data[0] & 0x07
        LLL = (data[0] >> 3) & 0x07
        if LLL == 0:
            data_len = 1
        else:
            data_len = 2 << (LLL - 1)
        if len(data) >= data_len+2:
            return data[1:1+data_len]
        else:
            return []

    def motor_set_type(self, id, value):
        if id in self.motor:
            self.motor[id].type = value
            self.write_32((id << 4 | 0x00), int(value))

    def _motor_get_type(self, id):
        self.read_32((id << 4 | 0x00))

    def motor_set_pulse(self, id, value):
        if id in self.motor:
            self.motor[id].pulse = value
            self.write_32((id << 4 | 0x01), int(value))

    def _motor_set_pulse(self, id, value):
        if id in self.motor:
            self.write_32((id << 4 | 0x01), int(value))

    def _motor_get_pulse(self, id):
        self.read_32((id << 4 | 0x01))

    def motor_set_point(self, id, value):
        self.write_32((id << 4 | 0x03), int(value))

    def motor_set_point_diff(self, id, value):
        if id in self.motor:
            position = self.motor[id].position + value
            self.write_32((id << 4 | 0x03), int(position))

    def motor_set_position(self, id, value):
        if id in self.motor:
            self.motor[id].position = value
            self.write_32((id << 4 | 0x02), int(value))

    def _motor_get_position(self, id):
        self.read_32((id << 4 | 0x02))

    def _sensor_get_type(self, id):
        id = remap_sensor_id(id)
        self.read_32((id+5) << 4 | 0x00)

    def _sensor_get_mode(self, id):
        id = remap_sensor_id(id)
        self.read_32(((id+5) << 4 | 0x01))

    def sensor_set_mode(self, id, value):
        id = remap_sensor_id(id)
        self.write_32(((id+5) << 4 | 0x01), int(value))

    def _sensor_get_value(self, id):
        id = remap_sensor_id(id)
        self.read_32(((id+5) << 4 | 0x02))

    def sensor_set_value(self, id, value):
        id = remap_sensor_id(id)
        self.write_32(((id+5) << 4 | 0x02), int(value))

    def motor_get_type(self, port):
        if port in self.motor:
            return self.motor[port].type
        else:
            return 0

    def motor_get_pulse(self, port):
        if port in self.motor:
            return self.motor[port].pulse
        else:
            return 0

    def motor_get_speed(self, port):
        if port in self.motor:
            return int(self.motor[port].pulse/655.35)
        else:
            return 0

    def motor_set_speed(self, port, speed):
        if port in self.motor:
            self.motor_set_pulse(port, int(speed*655.35))

    def motor_get_position(self, port):
        if port in self.motor:
            return self.motor[port].position
        else:
            return 0

    def sensor_get_type(self, port):
        # port = remap_sensor_id(port)
        if port in self.sensor:
            return self.sensor[port].type
        else:
            return 0

    def sensor_get_mode(self, port):
        # port = remap_sensor_id(port)
        if port in self.sensor:
            return self.sensor[port].mode
        else:
            return 0

    def sensor_get_value(self, port):
        # port = remap_sensor_id(port)
        if port in self.sensor:
            return self.sensor[port].value
        else:
            return 0

    def motor_get_info(self, port):
        if port in self.motor:
            return (port, self.motor_get_type(port), self.motor_get_pulse(port), self.motor_get_position(port))
        else:
            return (0, 0, 0, 0)

    def sensor_get_info(self, port):
        # port = remap_sensor_id(port)
        if port in self.sensor:
            return (port, self.sensor_get_type(port), self.sensor_get_mode(port), self.sensor_get_value(port))
        else:
            return (0, 0, 0, 0)

    def motor_set_angle(self, port, angle):
        """
        设置舵机角度

        Keyword arguments:
        port -- 端口号
        angle -- 角度 -90 到 90
        """
        # [0,180] => [-1550,-7450]
        if port in self.motor and abs(angle) <= 90:
            self.motor_set_pulse(port, int(4500+angle*32))

    def _system_get_version(self):
        self.read_32(0x00)

    def _system_get_power(self):
        self.read_32(0x01)

    def _system_get_charge(self):
        self.read_32(0x02)

    def system_get_version(self):
        return self.system.version

    def system_get_power(self):
        return self.system.battery_level

    def system_get_charge(self):
        return self.system.charge_state

    def system_poweroff(self):
        self.write_32(0x04, 0xFF)

    def system_get_power_meas(self):
        self.read_32(0x04)
        return [self.system.battery_mV, self.system.battery_mA, self.system.charge_state]

    def system_get_vout1(self):
        self.read_32(0x05)
        return [self.system.vout1_mV, self.system.vout1_mA, self.system.charge_state]

    def system_get_vout2(self):
        self.read_32(0x06)
        return [self.system.vout2_mV, self.system.vout2_mA, self.system.charge_state]


if __name__ == '__main__':
    import serial.tools.list_ports
    import math
    import random

    def pubSensorChange(port, sensor_id, status):
        print(port, sensor_id, status)

    serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
    print(serial_ports)
    driver = D51Driver('/dev/ttyACM0', baud_rate=1152000,
                       onSensorChange=pubSensorChange)
    # driver.system_poweroff()
    print(driver.port.is_open)
    count = 1
    time.sleep(3)
    # exit()
    while True:
        time.sleep(3)
        continue
        pulse = int(65535*math.sin(count/500.0*math.pi))
        # pulse = random.randint(0, 9)
        # print(pulse)
        driver.motor_set_pulse(1, pulse)
        driver.motor_set_pulse(2, pulse)
        driver.motor_set_pulse(3, pulse)
        driver.motor_set_pulse(4, pulse)
        driver.motor_set_pulse(5, pulse)
        count = count + 1
        time.sleep(0.01)
        if count > 1000:
            count = 0
