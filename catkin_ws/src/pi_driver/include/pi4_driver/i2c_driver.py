#!coding:utf-8
import time
import math

try:
    import smbus
    import RPi.GPIO as GPIO
except Exception as e:
    print(e)

from reg import *

ReadAccSensor = LSM6DSL_ACC_GYRO_OUTX_L_XL
ReadGyroSensor = LSM6DSL_ACC_GYRO_OUTX_L_G
ReadMagnetometer = BMM150_DATA_X_LSB


class I2cDriver:
    """
    I2cDriver 类, m031 i2c 驱动
    Attributes:
    m031_addr: hex m031 i2c地址
    int_pin: int m031 中断引脚
    bus: SMBus i2c总线
    btn_handler: function 按键中断函数
    """

    def __init__(self, btn_handler=None):
        self.LSM6DSL = 0x6a
        self.BMM150 = 0x10
        self.int_pin = 22  # GPIO25 40pin 第22号引脚
        self.bus = smbus.SMBus(1)
        self.senserData = {
            'acc': [0, 0, 0],
            'gryo': [0, 0, 0],
            'magn': [0, 0, 0],
        }
        self.offset = {
            "magn": [0, 0, 0]
        }
        self.nineAxisSetEnable()
        self.detectOffset()
        # self.detectOffset()
        if btn_handler is not None:
            self.btn_handler = btn_handler
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.int_pin, GPIO.IN)
            GPIO.add_event_detect(self.int_pin, GPIO.BOTH,
                                  callback=self.int_handler, bouncetime=20)

    def acc_set_enable(self, speed=100):
        if speed == 0:
            self.bus.write_byte_data(
                self.LSM6DSL, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x00)
        else:
            self.bus.write_byte_data(
                self.LSM6DSL, LSM6DSL_ACC_GYRO_CTRL1_XL, 0x60)
            self.bus.write_byte_data(
                self.LSM6DSL, LSM6DSL_ACC_GYRO_INT1_CTRL, 0x01)

    def gyro_set_enable(self, speed=100):
        if speed == 0:
            self.bus.write_byte_data(
                self.LSM6DSL, LSM6DSL_ACC_GYRO_CTRL2_G, 0x00)
        else:
            self.bus.write_byte_data(
                self.LSM6DSL, LSM6DSL_ACC_GYRO_CTRL2_G, 0x60)
            self.bus.write_byte_data(
                self.LSM6DSL, LSM6DSL_ACC_GYRO_INT1_CTRL, 0x02)

    def magn_set_enable(self, speed=100):
        if speed == 0:
            self.bus.write_byte_data(self.BMM150, BMM150_CONTROL, 0x3E)
        else:
            self.bus.write_byte_data(self.BMM150, BMM150_POWER_CONTROL, 0x01)
            time.sleep(0.1)
            self.bus.write_byte_data(self.BMM150, BMM150_CONTROL, 0x38)

    def nineAxisSetEnable(self, value=100):
        self.acc_set_enable(value)
        self.gyro_set_enable(value)
        self.magn_set_enable(value)

    def readSensorData(self, sensorType, byteLen=6):
        data = [0, 0, 0]
        maxTry = 5
        while maxTry > 0:
            try:
                if sensorType == ReadMagnetometer:
                    data = self.bus.read_i2c_block_data(
                        self.BMM150, sensorType, byteLen)
                    print(data[0], data[2], data[4])
                    x = (data[1] << 8 | data[0]) >> 3
                    y = (data[3] << 8 | data[2]) >> 3
                    z = (data[5] << 8 | data[4]) >> 1
                    data = [x, y, z]
                    break
                else:
                    data = self.bus.read_i2c_block_data(
                        self.LSM6DSL, sensorType, byteLen)
                    x = data[1] << 8 | data[0]
                    y = data[3] << 8 | data[2]
                    z = data[5] << 8 | data[4]
                    data = [x, y, z]
                    break
            except Exception as e:
                print(e)
                maxTry = maxTry - 1
        if data is None:
            if sensorType == ReadAccSensor:
                data = self.senserData['acc']
            elif sensorType == ReadGyroSensor:
                data = self.senserData['gyro']
            elif sensorType == ReadMagnetometer:
                data = self.senserData['magn']
        else:
            if sensorType == ReadAccSensor:
                data = self.normalizeSensorData(data)
                self.senserData['acc'] = data
            elif sensorType == ReadGyroSensor:
                data = self.normalizeSensorData(data)
                self.senserData['gyro'] = data
            elif sensorType == ReadMagnetometer:
                data = self.normalizeMagnData(data)
                self.senserData['magn'] = data
        return data

    def readOffsetSensorData(self, sensorType):
        data = self.readSensorData(sensorType)
        if sensorType == ReadAccSensor:
            offset = self.offset['acc']
            self.getOffsetData(data, offset)
        elif sensorType == ReadGyroSensor:
            offset = self.offset['gyro']
            self.getOffsetData(data, offset)
        elif sensorType == ReadMagnetometer:
            offset = self.offset['magn']
            self.getOffsetData(data, offset)
        return data

    def detectOffset(self):
        offset = {
            "acc": [0, 0, 0],
            "gyro": [0, 0, 0],
            "magn": self.offset['magn'],
        }
        for i in range(100):
            acc = self.readAccData()
            gyro = self.readGyroData()
            for i in range(3):
                offset['acc'][i] += acc[i]/100.0
                offset['gyro'][i] += gyro[i]/100.0
            print(offset)
        offset['acc'][2] = 16384 + offset['acc'][2]
        self.offset = offset

    def getOffset(self, sensor_id=0):
        if sensor_id == 1:
            return self.offset["acc"]
        elif sensor_id == 2:
            return self.offset['gyro']
        elif sensor_id == 3:
            return self.offset['magn']
        else:
            return [0, 0, 0]

    def setOffset(self, sensor_id=0, offset=[0, 0, 0]):
        if sensor_id == 1:
            self.offset["acc"] = offset
        elif sensor_id == 2:
            self.offset['gyro'] = offset
        elif sensor_id == 3:
            self.offset['magn'] = offset
        return offset

    def getOffsetData(self, data, offset):
        for i in range(3):
            data[i] = data[i] - offset[i]

    def normalizeSensorData(self, data):
        for i in range(3):
            if data[i] > 32768:
                data[i] = data[i] - 65536
        return data

    def normalizeMagnData(self, data):
        for i in range(2):
            if data[i] > 4096:
                data[i] = data[i] - 8192
        if data[2] > 4096:
            data[2] -= 32768
        return data

    def int_handler(self, channel):
        pass

    def readAccData(self, use_offset=False):
        if use_offset:
            return self.readOffsetSensorData(ReadAccSensor)
        else:
            return self.readSensorData(ReadAccSensor)

    def readGyroData(self, use_offset=False):
        if use_offset:
            return self.readOffsetSensorData(ReadGyroSensor)
        else:
            return self.readSensorData(ReadGyroSensor)

    def readMagnData(self, use_offset=False):
        if use_offset:
            return self.readOffsetSensorData(ReadMagnetometer)
        else:
            return self.readSensorData(ReadMagnetometer)

    def estimatePose(self):
        acc = self.readAccData(True)
        magn = self.readMagnData(True)
        acc_y = acc[1]/16384.0
        acc_x = -acc[0]/16384.0
        # 俯仰角 b
        if acc_y >= 1:
            pitch = math.pi/2
        elif acc_y <= -1:
            pitch = -math.pi/2
        else:
            pitch = math.asin(acc_y)
        # 横滚角 r
        roll = math.atan2(acc[0], -acc[2])
        # 方位角
        mx = magn[0]*math.cos(roll) - magn[2]*math.sin(roll)
        my = magn[0]*math.sin(pitch)*math.sin(roll)+magn[1] * \
            math.cos(pitch) - magn[2]*math.sin(pitch)*math.cos(roll)
        print(mx, my)
        if mx == 0 and my == 0:
            yaw = 0
        else:
            yaw = math.pi + math.atan2(mx, my)
        return [roll/math.pi*180, pitch/math.pi*180, yaw/math.pi*180]


if __name__ == '__main__':
    def test_print(data):
        print('read value %d ' % (data))
        return False
    # driver = I2cDriver(test_print)
    driver = I2cDriver()
    while True:
        print('acc:', driver.readAccData())
        print('acc:', driver.readAccData(True))
        print('gyro:', driver.readGyroData())
        print('gyro:', driver.readGyroData(True))
        print('magn:', driver.readMagnData())
        print('magn:', driver.readMagnData(True))
        print('pose:', driver.estimatePose())
        time.sleep(0.5)
