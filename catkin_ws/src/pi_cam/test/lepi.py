import spidev
import time

LP_SPI = spidev.SpiDev()
LP_SPI.open(0, 1)
LP_SPI.max_speed_hz = 500000
LP_SPI.mode = 0b00
LP_SPI.bits_per_word = 8

NO_ADDR = 0

# R/W   A6 A5 A4 A3	A2 A1 A0
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

class Motor(object):
	# A2 A1 A0 , Register Bit in Group
	TYPE = 0 # 0 for EV3 Motor
	ENABLE = 1 # 1 for enable, 0 for disable
	STATE = 2 
	POSITION = 3 # Current Position
	SETPOSITION = 4 # Target Position
	SPEED = 5 # from -65535 to 65535 , 0 means brake

class Message(object):
	@staticmethod
	def GetMotorState(port):
		return Command.READ | port | Motor.STATE
	@classmethod
	def GetCurrentPosition(port):
		return Command.READ | port | Motor.POSITION
	@classmethod
	def SetCurrentPosition(port):
		return Command.WRITE | port | Motor.POSITION
	@classmethod
	def GetTargetPosition(port):
		return Command.READ | port | Motor.SETPOSITION
	@classmethod
	def SetTargetPosition(port):
		return Command.WRITE | port | Motor.SETPOSITION
	@classmethod
	def GetMortorSpeed(port):
		return Command.READ | port | Motor.SPEED
	@staticmethod
	def SetMortorSpeed(port):
		return Command.WRITE | port | Motor.SPEED
	@staticmethod
	def SetMortorEnable(port):
		return Command.WRITE | port | Motor.ENABLE
class Lepi(object):
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
	Motors = {Group.MOTOR_1:1,Group.MOTOR_2:1,Group.MOTOR_3:1,Group.MOTOR_4:1,Group.MOTOR_5:1,}
	Sensors = {Group.SENSOR_1:1,Group.SENSOR_2:1,Group.SENSOR_3:1,Group.SENSOR_4:1,Group.SENSOR_5:1,}
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
		reply = LP_SPI.xfer2(outArray)
		return int((reply[5] << 24) | (reply[4] << 16) | (reply[3] << 8) | reply[2])

	@classmethod
	def spi_write_32(self, MessageType, Value):
		"""
		Send a 32-bit value over SPI

		Keyword arguments:
		MessageType -- the SPI message type
		Value -- the value to be sent
		"""
		outArray = [MessageType, (Value & 0xFF), ((Value >> 8) & 0xFF), ((Value >> 16) & 0xFF), ((Value >> 24) & 0xFF)]
		#print(outArray)
		LP_SPI.xfer2(outArray)
	@classmethod
	def motor_enable(self,port):
		if self.Motors.__contains__(port):
			self.spi_write_32(Message.SetMortorEnable(port),1)
		else:
			print('wrong motor number')
	@classmethod
	def motor_disable(self,port):
		if self.Motors.__contains__(port):
			self.spi_write_32(Message.SetMortorEnable(port),0)
		else:
			print('wrong motor number')
	@classmethod
	def motor_set_speed(self,port,speed):
		if self.Motors.__contains__(port):
			self.spi_write_32(Message.SetMortorSpeed(port),speed)
		else:
			print('wrong motor number')


# print(Command.WRITE | Group.MOTOR_3 | Motor.SPEED)

if __name__ == '__main__':
	import time
	Lepi.motor_enable(Group.MOTOR_1)
	Lepi.motor_enable(Group.MOTOR_2)
	Lepi.motor_enable(Group.MOTOR_3)
	Lepi.motor_enable(Group.MOTOR_4)
	Lepi.motor_set_speed(Group.MOTOR_1,50000)
	Lepi.motor_set_speed(Group.MOTOR_2,50000)
	Lepi.motor_set_speed(Group.MOTOR_3,50000)
	Lepi.motor_set_speed(Group.MOTOR_4,50000)
	time.sleep(1)
	Lepi.motor_disable(Group.MOTOR_1)
	Lepi.motor_disable(Group.MOTOR_2)
	Lepi.motor_disable(Group.MOTOR_3)
	Lepi.motor_disable(Group.MOTOR_4)
