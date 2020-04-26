#!coding:utf-8
import spidev
import time
import ctypes
import RPi.GPIO as GPIO
import numpy as np

LP_SPI = spidev.SpiDev()
LP_SPI.open(0, 1)
LP_SPI.max_speed_hz = 50000
LP_SPI.mode = 0b00
LP_SPI.bits_per_word = 8

NO_ADDR = 0

ERROR_PORT = -101

# R/W   A6 A5 A4 A3    A2 A1 A0


class Command(object):
    # R/W Command Bit
    WRITE = 0x00
    READ = 0x01 << 7


class Group(object):
    # A6 A5 A4 A3 , Group Register Bit
    # 0 for System,1-5 for Motor,6-10 for Sensor
    SYSTEM = 0x00 << 3

    MOTOR_1 = 0x01 << 3
    MOTOR_2 = 0x02 << 3
    MOTOR_3 = 0x03 << 3
    MOTOR_4 = 0x04 << 3
    MOTOR_5 = 0x05 << 3

    SENSOR_1 = 0x06 << 3
    SENSOR_2 = 0x07 << 3
    SENSOR_3 = 0x08 << 3
    SENSOR_4 = 0x09 << 3
    SENSOR_5 = 0x0a << 3

    SERVO = 0x0b << 3

class Motor(object):
    # A2 A1 A0 , Register Bit in Group
    TYPE = 0  # 0 for EV3 Motor
    ENABLE = 1  # 1 for enable, 0 for disable
    STATE = 2
    POSITION = 3  # Current Position
    SETPOSITION = 4  # Target Position
    SPEED = 5  # from -65535 to 65535 , 0 means brake


class Sensor(object):
    # A2 A1 A0 , Register Bit in Group
    TYPE = 0  # R/W 0:No Sensor , 29:EV3 Color Sensor , 30:EV3 Ultrasonic Sensor
    MODE = 1  # R/W do not support set work mode yet
    VALUE = 2  # R

class SERVO(object):
    # A2 A1 A0 , Register Bit in Group
    LEN = 0  # R 0:No Sensor , 29:EV3 Color Sensor , 30:EV3 Ultrasonic Sensor
    Rx = 1  # R do not support set work mode yet
    Tx = 2  # W
    PING = 1
    READ_DATA = 2
    WRITE_DATA = 3
    RESET = 6

class EEPROM(object):
    ID = 0x05
    MIN_POSITION_H = 0x09
    MIN_POSITION_L = 0x0a
    MAX_POSITION_H = 0x0b
    MAX_POSITION_L = 0x0c
    TARGET_POSITION_H = 0x2a
    TARGET_POSITION_L = 0x2b
    SPEED_H = 0x2e
    SPEED_L = 0x2f
    LOCK = 0x30
    CURRENT_POSITION_H = 0x38
    CURRENT_POSITION_L = 0x39

class System(object):
    SENSOR_STATUS = 0
    SENSOR_1 = 0x01 << 16
    SENSOR_2 = 0x01 << 17
    SENSOR_3 = 0x01 << 18
    SENSOR_4 = 0x01 << 19
    SENSOR_5 = 0x01 << 20
    Sensors = {1: SENSOR_1, 2: SENSOR_2, 3: SENSOR_3, 4: SENSOR_4, 5: SENSOR_5}

class Message(object):
    @staticmethod
    def GetMotorType(port):
        return Command.READ | port | Motor.TYPE

    @staticmethod
    def SetMotorType(port):
        return Command.WRITE | port | Motor.TYPE

    @staticmethod
    def GetMotorState(port):
        return Command.READ | port | Motor.STATE

    @staticmethod
    def SetMotorState(port):
        return Command.WRITE | port | Motor.STATE

    @staticmethod
    def GetCurrentPosition(port):
        return Command.READ | port | Motor.POSITION

    @staticmethod
    def SetCurrentPosition(port):
        return Command.WRITE | port | Motor.POSITION

    @staticmethod
    def GetTargetPosition(port):
        return Command.READ | port | Motor.SETPOSITION

    @staticmethod
    def SetTargetPosition(port):
        return Command.WRITE | port | Motor.SETPOSITION

    @staticmethod
    def GetMotorSpeed(port):
        return Command.READ | port | Motor.SPEED

    @staticmethod
    def SetMortorSpeed(port):
        return Command.WRITE | port | Motor.SPEED

    @staticmethod
    def SetMortorEnable(port):
        return Command.WRITE | port | Motor.ENABLE

    @staticmethod
    def GetMotorEnable(port):
        return Command.READ | port | Motor.ENABLE

    @staticmethod
    def GetSensorType(port):
        return Command.READ | port | Sensor.TYPE

    @staticmethod
    def GetSensorMode(port):
        return Command.READ | port | Sensor.MODE

    @staticmethod
    def GetSensorValue(port):
        return Command.READ | port | Sensor.VALUE

    @staticmethod
    def GetSensorStatus():
        return Command.READ | System.SENSOR_STATUS

    @staticmethod
    def GetServoRxLen():
        return Command.READ | Group.SERVO  |  SERVO.LEN

    @staticmethod
    def GetServoRx():
        return Command.READ | Group.SERVO | SERVO.Rx

    @staticmethod
    def SetServoTx():
        return Command.WRITE | Group.SERVO | SERVO.Tx

def chk_sum(data):
    data[-1] = 0
    # print(data)
    check = ( ~(sum(data) & 0xFF) ) & 0xFF
    return check

class Lepi(object):
    """
    Lepi 类, d51 spi 驱动
    Attributes:
    spi: SPI d51 spi总线
    int_pin: int d51 中断引脚
    """
    SYSTEM = 0x00 << 3
    MOTOR_1 = 1
    MOTOR_2 = 2
    MOTOR_3 = 3
    MOTOR_4 = 4
    MOTOR_5 = 5
    SENSOR_1 = 1
    SENSOR_2 = 2
    SENSOR_3 = 3
    SENSOR_4 = 4
    SENSOR_5 = 5
    Motors = {1: Group.MOTOR_1, 2: Group.MOTOR_2,
              3: Group.MOTOR_3, 4: Group.MOTOR_4, 5: Group.MOTOR_5}
    Sensors = {1: Group.SENSOR_1, 2: Group.SENSOR_2,
               3: Group.SENSOR_3, 4: Group.SENSOR_4, 5: Group.SENSOR_5}
    spi = LP_SPI
    @classmethod
    def spi_read_32(self, MessageType):
        """
        Read a 32-bit value over SPI

        Keyword arguments:
        MessageType -- the SPI message type

        Returns :
        value
        """
        outArray = [MessageType, 0, 0, 0, 0, 0]
        reply = self.spi.xfer2(outArray)
        # print(reply)
        cint32 = ctypes.c_int32((reply[5] << 24) | (
            reply[4] << 16) | (reply[3] << 8) | reply[2])
        return cint32.value
        # value = int((reply[5] << 24) | (reply[4] << 16) | (reply[3] << 8) | reply[2])
        # print(value)
        # if value >> 31 == 1:
        #     value = value - 0x100000000
        # return value

    @classmethod
    def spi_write_32(self, MessageType, Value):
        """
        Send a 32-bit value over SPI

        Keyword arguments:
        MessageType -- the SPI message type
        Value -- the value to be sent
        """
        outArray = [MessageType, (Value & 0xFF), ((Value >> 8) & 0xFF), ((
            Value >> 16) & 0xFF), ((Value >> 24) & 0xFF)]
        # print(outArray[1:5])
        self.spi.xfer2(outArray)
        time.sleep(0.004)
        return 1

    @classmethod
    def motor_get_type(self, port):
        """
        获取电机类型，电机 or 舵机

        Keyword arguments:
        port -- 端口号
        """
        if self.Motors.has_key(port):
            return self.spi_read_32(Message.GetMotorType(self.Motors[port]))
        return ERROR_PORT

    @classmethod
    def motor_set_type(self, port, value):
        """
        设置电机类型，电机 or 舵机

        Keyword arguments:
        port -- 端口号
        value -- 0代表电机 1代表舵机
        """
        if self.Motors.has_key(port):
            # print(port,value)
            return self.spi_write_32(Message.SetMotorType(self.Motors[port]), value)
        return ERROR_PORT

    @classmethod
    def motor_get_state(self, port):
        if self.Motors.has_key(port):
            return self.spi_read_32(Message.GetMotorState(self.Motors[port]))
        return ERROR_PORT

    @classmethod
    def motor_set_state(self, port, value):
        """
        设置电机状态

        Keyword arguments:
        port -- 端口号
        value -- 电机状态
        """
        if self.Motors.has_key(port):
            return self.spi_write_32(Message.SetMotorState(self.Motors[port]), value)
        return ERROR_PORT

    @classmethod
    def motor_set_enable(self, port, value):
        if self.Motors.has_key(port):
            return self.spi_write_32(Message.SetMortorEnable(self.Motors[port]), value)
        return ERROR_PORT

    @classmethod
    def motor_get_enable(self, port):
        if self.Motors.has_key(port):
            return self.spi_read_32(Message.GetMotorEnable(self.Motors[port]))
        return ERROR_PORT

    @classmethod
    def motor_set_speed(self, port, speed):
        """
        设置电机速度

        Keyword arguments:
        port -- 端口号
        speed -- 速度 -100 到 100
        """
        if self.Motors.has_key(port):
            self.spi_write_32(Message.SetMortorSpeed(
                self.Motors[port]), int(speed*655.35))
        return ERROR_PORT

    @classmethod
    def motor_set_pulse(self, port, speed):
        if self.Motors.has_key(port):
            self.spi_write_32(Message.SetMortorSpeed(
                self.Motors[port]), speed)
        return ERROR_PORT

    @classmethod
    def motor_get_current_position(self, port):
        if self.Motors.has_key(port):
            return self.spi_read_32(Message.GetCurrentPosition(self.Motors[port]))
        return ERROR_PORT

    @classmethod
    def motor_set_target_position(self, port, position):
        if self.Motors.has_key(port):
            self.spi_write_32(Message.SetTargetPosition(
                self.Motors[port]), position)
        return ERROR_PORT

    @classmethod
    def motor_get_target_position(self, port):
        if self.Motors.has_key(port):
            self.spi_read_32(Message.GetTargetPosition(self.Motors[port]))
        return ERROR_PORT

    @classmethod
    def motor_get_speed(self, port):
        if self.Motors.has_key(port):
            return int(self.spi_read_32(Message.GetMotorSpeed(self.Motors[port]))/655.35)
        return ERROR_PORT

    @classmethod
    def motor_get_pulse(self, port):
        if self.Motors.has_key(port):
            return int(self.spi_read_32(Message.GetMotorSpeed(self.Motors[port])))
        return ERROR_PORT

    @classmethod
    def motor_get_info(self, port):
        if self.Motors.has_key(port):
            return (port, self.motor_get_type(port), self.motor_get_pulse(port), self.motor_get_current_position(port))
        return ERROR_PORT

    @classmethod
    def motor_set_info(self, port, enable, speed):
        if self.Motors.has_key(port):
            self.motor_set_type(port, enable)
            self.motor_set_speed(port, speed)
        return ERROR_PORT

    @classmethod
    def sensor_get_type(self, port):
        if self.Sensors.has_key(port):
            return self.spi_read_32(Message.GetSensorType(self.Sensors[port]))
        return ERROR_PORT

    @classmethod
    def sensor_get_mode(self, port):
        if self.Sensors.has_key(port):
            return self.spi_read_32(Message.GetSensorMode(self.Sensors[port]))
        return ERROR_PORT

    @classmethod
    def sensor_get_value(self, port):
        if self.Sensors.has_key(port):
            return self.spi_read_32(Message.GetSensorValue(self.Sensors[port]))
        return ERROR_PORT
    @classmethod
    def sensor_get_info(self, port):
        if self.Sensors.has_key(port):
            return (port, self.sensor_get_type(port), self.sensor_get_mode(port), self.sensor_get_value(port))
        return ERROR_PORT
    @classmethod
    def system_get_sensor_status(self):
        status = self.spi_read_32(Message.GetSensorStatus())
        print(status)
        return status

    @classmethod
    def motor_set_angle(self, port, angle):
        """
        设置舵机角度

        Keyword arguments:
        port -- 端口号
        angle -- 角度 -90 到 90
        """
        # [0,180] => [-1550,-7450]
        if self.Motors.has_key(port) and abs(angle) <= 90:
            self.spi_write_32(Message.SetMortorSpeed(
                self.Motors[port]), int(-4500+angle*32))
        return ERROR_PORT

    @classmethod
    def servo_ping(self,id):
        data = [Message.SetServoTx(),0xff,0xff,id,0x02,0x01,0]
        data[-1] = chk_sum(data[3:])
        # print('transfer:',data)
        self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        status = Lepi.servo_rx_data(l)
        print('ping',id,status)
        if(len(status) == 6 ):
            return True
        else:
            return False

    @classmethod
    def servo_reset(self,id):
        data = [Message.SetServoTx(),0xff,0xff,id,0x02,SERVO.RESET,0]
        data[-1] = chk_sum(data[3:])
        # print('transfer:',data)
        self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        status = Lepi.servo_rx_data(l)
        print('servo_reset',id,status)

    @classmethod
    def servo_get_position(self,id):
        data = [Message.SetServoTx(),0xff,0xff ,id ,0x04 ,0x02 ,0x38 ,0x02 ,0]
        # data = [FF FF 01 09 03 2A 00 08 00 00 E8 03 D5]
        # data = [Message.SetServoTx(),0xff,0xff ,0x01,0x09,0x03,0x2a,0x00,0x08,0x00,0x00,0xe8,0x03,0x00]
        # chk = chk_sum(data)
        data[-1] = chk_sum(data[3:])
        # print('transfer:' ,''.join(format(x, '02x') for x in data))
        # print(data[3:])
        status = self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        res = Lepi.servo_rx_data(l)
        if(len(res) == 8):
            return (res[-3]<<8) + res[-2]
        else:
            print('data error: receive ',res)
            return 0


    @classmethod
    def servo_set_position(self,id,position,ms=0,speed=0):
        # 0 <= min_position <= position <= max_position <= 0x03ff (200 degree)
        # 0 <= min_speed <= speed <= max_speed <= 0x03ff
        if(position <0):
            position = 0
        if(position > 0x03ff):
            position = 0x03ff
        if(speed <0):
            speed = 0
        if(speed > 0x03ff):
            speed = 0x03ff
        if(ms <0):
            ms = 0
        if(ms > 0x03ff):
            ms = 0x03ff
        data = [Message.SetServoTx(),0xff,0xff ,id ,0x09 ,0x03 ,0x2A ,(position >> 8) & 0xFF ,position & 0xFF ,(ms >> 8) & 0xFF ,ms & 0xFF,(speed >> 8) & 0xFF ,speed & 0xFF ,0]
        data[-1] = chk_sum(data[3:])
        print(data)
        print('transfer:' ,''.join(format(x, '02x') for x in data))
        self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        status = Lepi.servo_rx_data(l)
        print('servo_set_position',id,position,ms,speed,status)
        return l

    @classmethod
    def servo_rx_len(self):
        res = self.spi_read_32(Message.GetServoRxLen())
        # print('receive:',res)
        return res

    @classmethod
    def servo_rx_data(self,count):
        if(count<2):
            return [0]
        data = [0 for i in range(count+2)]
        data[0] = Message.GetServoRx()
        # print('transfer:',data)
        status = self.spi.xfer2(data)
        # print('receive:',status)
        # print('chk_sum:',chk_sum(status[4:]))
        return status[2:]
        # print(Command.WRITE | Lepi.MOTOR_3 | Motor.SPEED)

    @classmethod
    def servo_scan(self):
        devices = []
        for i in range(254):
            if(self.servo_ping(i)):
                devices.append(i)
        return devices

    @classmethod
    def servo_set_id(self,id_old,id_new):
        data = [Message.SetServoTx(),0xff,0xff,id_old,0x04,SERVO.WRITE_DATA,EEPROM.ID,id_new,0]
        data[-1] = chk_sum(data[3:])
        print('transfer:' ,''.join(format(x, '02x') for x in data))
        self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        status = Lepi.servo_rx_data(l)
        print('set_id',id_old,id_new,status)
        return l

    @classmethod
    def servo_write_u8(self,id,param,value):
        data = [Message.SetServoTx(),0xff,0xff,id,0x04,SERVO.WRITE_DATA,param,value,0]
        data[-1] = chk_sum(data[3:])
        print('transfer:' ,''.join(format(x, '02x') for x in data))
        self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        status = Lepi.servo_rx_data(l)
        print('servo_write_u8',id,param,value,status)
        return l
    @classmethod
    def servo_write_u16(self,id,param,value):
        data = [Message.SetServoTx(),0xff,0xff,id,0x05,SERVO.WRITE_DATA,param,(value >> 8) & 0xff,value & 0xff,0]
        data[-1] = chk_sum(data[3:])
        print('transfer:' ,''.join(format(x, '02x') for x in data))
        self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        status = Lepi.servo_rx_data(l)
        print('servo_write_u16',id,param,value,status)
        return l

    @classmethod
    def servo_read_u8(self,id,param):
        data = [Message.SetServoTx(),0xff,0xff,id,0x04,SERVO.READ_DATA,param,0x01,0]
        data[-1] = chk_sum(data[3:])
        print('transfer:' ,''.join(format(x, '02x') for x in data))
        self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        res = Lepi.servo_rx_data(l)
        print('servo_read_u8',id,param,res)
        if(len(res) == 7) :
            return (res[-3]<<8) + res[-2]
        else:
            print('data error: receive ',res)
            return 0
    @classmethod
    def servo_read_u16(self,id,param):
        # print(id,param)
        data = [Message.SetServoTx(),0xff,0xff,id,0x04,SERVO.READ_DATA,param,0x02,0]
        data[-1] = chk_sum(data[3:])
        # print('transfer:' ,''.join(format(x, '02x') for x in data))
        self.spi.xfer2(data)
        time.sleep(0.002)
        l = Lepi.servo_rx_len()
        res = Lepi.servo_rx_data(l)
        # print('servo_read_u16',id,param,res)
        if(len(res) == 8) :
            return (res[-3]<<8) + res[-2]
        else:
            print('data error: receive ',res)
            return 0

class D51Driver:
    """
    D51Driver 类, d51 spi 驱动
    Attributes:
    int_pin: int d51 中断引脚
    int_handler: function 中断函数
    onSensorChange: function 传感器状态改变回调函数
    sensor_type: {sensor_id:sensor_name} 传感器类型
    sensors: [sensor_id] 传感器连接状态

    """

    def __init__(self, onSensorChange=None):
        self.int_pin = 31  # GPIO06 40pin 第31号引脚
        self.int_handler = self.defaultHandler
        self.onSensorChange = onSensorChange
        self.sensor_type = {0: 'Sensor',
                            29: 'Infrared Sensor', 30: 'Ultrasonic Sensor'}
        self.sensors = self.readSensorStatus()
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.int_pin, GPIO.IN)
        GPIO.add_event_detect(self.int_pin, GPIO.RISING,
                              callback=self.int_handler, bouncetime=20)

    def readSensorStatus(self):
        status = Lepi.system_get_sensor_status()
        sensors = []
        for k, v in System.Sensors.items():
            sensors.append(int(status & v != 0))
        return np.array(sensors)

    def defaultHandler(self, channel):
        """
        默认中断处理函数

        Keyword arguments:
        channel -- 中断频道
        change -- 传感器状态是否改变
        sensors: [sensor_id] 传感器连接状态
        """
        print('sensor status changed', channel)
        sensors = self.readSensorStatus()
        change = sensors - self.sensors
        self.sensors = sensors
        print(change)
        for i, status in enumerate(change):
            sensor_type = Lepi.sensor_get_type(i+1)
            if status == 1:
                print(i+1, self.sensor_type[sensor_type], 'connected')
            elif status == -1:
                print(i+1, self.sensor_type[sensor_type], 'disconnected')
            else:
                continue
            if self.onSensorChange != None:
                self.onSensorChange(i+1, sensor_type, status)


if __name__ == '__main__':
    # test_servo(Lepi.MOTOR_1)
    # driver = D51Driver()
    # while True:
    #     time.sleep(1)
    # test_motor()
    # test_sensor()
    # print(Lepi.servo_get_position(1))
    # # time.sleep(2)
    # print(Lepi.servo_get_position(1))
    print((Lepi.servo_scan()))
    # print(Lepi.servo_read_u8(1,EEPROM.LOCK))
    # Lepi.servo_write_u8(1,EEPROM.LOCK,0)
    # print(Lepi.servo_read_u8(1,EEPROM.LOCK))
    # Lepi.servo_set_id(1,2)
    # print(Lepi.servo_read_u8(2,EEPROM.LOCK))
    # Lepi.servo_write_u8(2,EEPROM.LOCK,1)
    # print(Lepi.servo_read_u8(2,EEPROM.LOCK))
    # Lepi.servo_ping(2)
    # print(Lepi.servo_write_u16(2,EEPROM.MAX_POSITION_H,1000))
    # print(Lepi.servo_write_u16(2,EEPROM.MIN_POSITION_H,0))
    # Lepi.servo_set_position(2,0,1000,200)
    # print(Lepi.servo_write_u16(2,0x2c,200))
    # print(Lepi.servo_read_u16(2,EEPROM.MIN_POSITION_H))
    # print(Lepi.servo_read_u16(2,EEPROM.MAX_POSITION_H))