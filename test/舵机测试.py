#!coding:utf-8
from pi_driver import SServo
import time
import os

if __name__ == '__main__':
    import serial.tools.list_ports
    serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
    print(serial_ports)
    # servo = SServo('/dev/ttyAMA1')
    servo = SServo()
    while True:
        os.system('clear')
        ids = servo.scan_print(25)
        for i in ids:
            servo.set_position(i, 50, speed=2000)
        time.sleep(0.7)
        for i in ids:
            servo.set_position(i, 1023/2, speed=2000)
        time.sleep(0.7)
        for i in ids:
            servo.set_position(i, 1950, speed=2000)
        time.sleep(0.7)
        for i in ids:
            servo.set_position(i, 1023/2, speed=2000)
        time.sleep(0.7)
