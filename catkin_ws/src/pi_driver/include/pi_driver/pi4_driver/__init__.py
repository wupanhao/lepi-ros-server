from .i2c_driver import *
from .d51_driver import D51Driver
from .sservo import *
from .buttonListener import ButtonListener
from pi_driver.srv import GetInt32, GetInt32Request


class LepiDriverInterface:
    def __init__(self):
        pass

    def motor_set_type(self):
        pass

    def motor_set_angle(self):
        pass

    def motor_set_state(self):
        pass

    def motor_set_enable(self):
        pass

    def motor_set_speed(self):
        pass

    def motor_get_position(self):
        pass

    def motor_set_position(self):
        pass

    def servo_set_position(self):
        pass

    def servo_write_u16(self):
        pass

    def servo_write_u8(self):
        pass

    def servo_read_u16(self):
        pass

    def servo_read_u8(self):
        pass

    def nineAxisSetEnable(self):
        pass

    def readAccData(self):
        pass

    def readGyroData(self):
        pass

    def readMagnData(self):
        pass

    def sensor_get_value(self):
        pass


class LepiDriver:
    def __init__(self):
        import rospy
        try:
            rospy.init_node("pi_driver_client")
        except Exception as e:
            pass
        self.srv_sensor_get_value = rospy.ServiceProxy(
            '/ubiquityrobot/pi_driver_node/sensor_get_value', GetInt32)
        self.d51_driver = D51Driver()
        self.i2c_driver = I2cDriver()

    def motor_set_type(self, port, value):
        self.d51_driver.motor_set_type(port, value)

    def motor_set_angle(self, port, value):
        self.d51_driver.motor_set_angle(port, value)

    def motor_set_state(self, port, value):
        # self.d51_driver.motor_set_type(port,value)
        pass

    def motor_set_enable(self, port, value):
        # self.d51_driver.motor_set_type(port,value)
        pass

    def motor_set_speed(self, port, value):
        self.d51_driver.motor_set_speed(port, value)

    def motor_get_position(self, port):
        return self.d51_driver.motor_get_position(port)

    def motor_set_position(self, port, value):
        self.d51_driver.motor_set_position(port, value)

    def servo_set_position(self, port, value):
        # self.d51_driver.servo_set_position(port, value)
        pass

    def servo_write_u16(self):
        pass

    def servo_write_u8(self):
        pass

    def servo_read_u16(self):
        pass

    def servo_read_u8(self):
        pass

    def nineAxisSetEnable(self, value):
        self.i2c_driver.nineAxisSetEnable(value)

    def readAccData(self):
        return self.i2c_driver.readAccData(True)

    def readGyroData(self):
        return self.i2c_driver.readGyroData(True)

    def readMagnData(self):
        return self.i2c_driver.readMagnData(True)

    def sensor_get_value(self, port):
        resp = self.srv_sensor_get_value(GetInt32Request(port))
        return resp.value
        # return self.d51_driver.sensor_get_value(port)
