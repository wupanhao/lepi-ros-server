# import pigpio
import serial.tools.list_ports
from .Config import ServoParams, PWMParams

#!coding:utf-8
import serial
import time
import math

from pi_driver import SServo

header = [0x12, 0x4c]


def chk_sum(data):
    # data[-1] = 0
    # print(data)
    check = sum(data) % 256
    return check


class SERVO(object):
    PING = 1
    READ_DATA = 2
    WRITE_DATA = 3
    RESET = 6


class Servo:
    def __init__(self, id, position, ms=0, speed=0):
        self.id = id
        self.position = position
        self.ms = ms
        self.speed = speed


class SServo2(object):
    """
    serial client
    """

    def __init__(self, port, baud_rate=115200):
        self.port = serial.Serial(port=port, baudrate=baud_rate, bytesize=8, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE, timeout=0.01)

    def send_str(self, cmd):
        self.port.write(cmd.encode('utf-8'))

    def read_str(self):
        response = self.port.readline()
        return response.decode('utf-8')

    def read_num(self, num):
        response = self.port.read(num)
        return response

    def send_hex(self, cmd):
        # print(cmd)
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
            array.append(i)
        return array

    def ping(self, id):
        data = [header[0], header[1], 0x01, 0x01, id]
        data.append(chk_sum(data))
        self.send_hex(data)
        res = self.read_hex(6)
        # print(res)
        if len(res) >= 6 and res[4] == id:
            return True
        else:
            return False

    def set_position(self, id, angle, ms=10, pw=0):
        data = [header[0], header[1], 0x08, 0x07, id, angle & 0xff,
                (angle >> 8) & 0xff, ms & 0xff, (ms >> 8) & 0xff, pw & 0xff, (pw >> 8) & 0xff]
        data.append(chk_sum(data))
        self.send_hex(data)
        # res = self.read_hex()
        # print(res)

    def set_positions(self, ids, angles, ms=1, pw=0):
        array = []
        for i in range(len(ids)):
            data = [header[0], header[1], 0x08, 0x07, ids[i], angles[i] & 0xff,
                    (angles[i] >> 8) & 0xff, ms & 0xff, (ms >> 8) & 0xff, pw & 0xff, (pw >> 8) & 0xff]
            data.append(chk_sum(data))
            array.extend(data)
        print(array)
        self.send_hex(array)

    def get_position(self, id):
        data = [header[0], header[1], 0x0a, 0x01, id]
        data.append(chk_sum(data))
        self.send_hex(data)
        res = self.read_hex()
        print(res)

    def set_id(self, id_from, id_to):
        data = [header[0], header[1], 0x04, 0x03, id_from, 34, id_to]
        data.append(chk_sum(data))
        self.send_hex(data)
        res = self.read_hex()
        print(res)

    def read_param(self, id, param_id):
        data = [header[0], header[1], 0x03, 0x02, id, param_id]
        data.append(chk_sum(data))
        self.send_hex(data)
        res = self.read_hex()
        print(res)

    def set_param(self, id, param_id, param_value):
        data = [header[0], header[1], 0x04, 0x04, id, param_id,
                param_value & 0xff, (param_value >> 8) & 0xff]
        data.append(chk_sum(data))
        self.send_hex(data)
        res = self.read_hex()
        print(res)


serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
print(serial_ports)


class HardwareInterface:
    def __init__(self):
        # self.pi = pigpio.pi()
        self.pi = SServo(serial_ports[0])
        self.speed = 2000
        # self.pwm_params = PWMParams()
        self.servo_params = ServoParams()
        self.servo_ids = [i+1 for i in range(12)]
        self.servo_angles = [0 for i in range(12)]
        # self.test()
        # self.change_power()
        # initialize_pwm(self.pi, self.pwm_params)

    def test(self):
        import time
        self.set_actuator_angles(
            [24, -22, -3, 11, -8, 7, -13, -11, 10, 5, 0, 10])
        time.sleep(2)
        self.set_actuator_angles(
            [26, 2, 23, 9, -33, -19, -11, 13, 37, 3, -25, -16])
        time.sleep(2)
        self.set_actuator_angles(
            [19, -39, -25, 16, 8, 29, -18, -28, -11, 10, 16, 32])

    def change_power(self):
        for i in range(12):
            self.pi.set_param(i+1, 42, 20000)

    def set_actuator_postions(self, joint_angles):
        angles = []
        ids = []
        leg_offset = [0, 45, -45]
        for leg_index in range(4):
            for axis_index in range(3):
                id = leg_index*3+axis_index+1
                ids.append(id)
                angle = -(joint_angles[axis_index][leg_index]/math.pi*180 - leg_offset[axis_index]
                          ) * self.servo_params.servo_multipliers[axis_index, leg_index]
                self.servo_angles[id-1] = int(angle)
                angle = angle - \
                    self.servo_params.neutral_angle_degrees[axis_index][leg_index]
                angles.append(int(angle))
                # print(angle)
        servos = [Servo(ids[i], int(angles[i]/300.0*1023+1023/2), speed=self.speed)
                  for i in range(12)]
        self.pi.set_positions_sync(servos)
        return ids, angles

    def set_actuator_angles(self, angles):
        # angles = []
        ids = []
        for leg_index in range(4):
            for axis_index in range(3):
                id = leg_index*3+axis_index+1
                ids.append(id)
                angles[id-1] = angles[id-1] - \
                    self.servo_params.neutral_angle_degrees[axis_index][leg_index]
        servos = [Servo(ids[i], int(angles[i]/300.0*1023+1023/2), speed=2000)
                  for i in range(12)]
        self.pi.set_positions_sync(servos)

    def set_actuator_position(self, joint_angle, axis, leg):
        send_servo_command(self.pi, self.pwm_params,
                           self.servo_params, joint_angle, axis, leg)


def deactivate_servos(pi, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            pi.set_PWM_dutycycle(pwm_params.pins[axis_index, leg_index], 0)


if __name__ == '__main__':
    hardwareInterface = HardwareInterface()
