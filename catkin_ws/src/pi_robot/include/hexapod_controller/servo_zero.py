#!coding:utf-8
from sservo import SServo
import time
import os

if __name__ == '__main__':
    import serial.tools.list_ports
    serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
    print(serial_ports)
    # servo = SServo('/dev/ttyAMA1')
    servo = SServo()
    while True:
        # os.system('clear')
        # servo.get_offset(1)
        # time.sleep(1)
        # servo.set_offset(1, 0)
        # time.sleep(1)
        servo.set_position(1, 500, ms=100)
        print(servo.get_offset(1))
        print(servo.get_position(1))
        # print(servo.read_u8(1,0x28))
        # time.sleep(1)
        servo.toggle_force(1)
        # servo.toggle_movable(1)
        break
