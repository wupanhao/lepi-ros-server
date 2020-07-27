#!coding:utf-8
import time

try:
  import smbus
  import RPi.GPIO as GPIO
except Exception as e:
  print(e)
ReadButtonState = 0x82

ReadAccSensor = 0x83
ReadGyroSensor = 0x84
ReadMagnetometer = 0x85
ReadBatOcv = 0x8A

WriteNineAxis = 0x46

OpenAccSensor = 0x44
OpenGyroSensor = 0x42
OpenMagnetometer = 0x41
OpenNineAxis = 0x47

Button = {
    0x81: 'btn1down',
    0x82: 'btn2down',
    0x83: 'btn3down',
    0x84: 'btn4down',
    0x85: 'btn5down',
    0x86: 'btn6down',
    0x87: 'btn7down',
    0x88: 'btn8down',
    0x89: 'btn9down',

    0x01: 'btn1up',
    0x02: 'btn2up',
    0x03: 'btn3up',
    0x04: 'btn4up',
    0x05: 'btn5up',
    0x06: 'btn6up',
    0x07: 'btn7up',
    0x08: 'btn8up',
    0x09: 'btn9up',

    0x11: 'btn1short',
    0x12: 'btn2short',
    0x13: 'btn3short',
    0x14: 'btn4short',
    0x15: 'btn5short',
    0x16: 'btn6short',
    0x17: 'btn7short',
    0x18: 'btn8short',
    0x19: 'btn9short',

    0x91: 'btn1long',
    0x92: 'btn2long',
    0x93: 'btn3long',
    0x94: 'btn4long',
    0x95: 'btn5long',
    0x96: 'btn6long',
    0x97: 'btn7long',
    0x98: 'btn8long',
    0x99: 'btn9long',

    0xa9: 'shutDownRequest',
    0x55: 'PowerInfoChanged'
}

ButtonMap = {
    # type==1
    # button down
    0x81: 77,  # Menu 'M' 77
    0x82: 38,  # 'ArrowUp': 38
    0x83: 66,  # Back 'B' 66
    0x84: 37,  # 'ArrowLeft': 37
    0x85: 13,  # 'Enter': 13
    0x86: 39,  # 'ArrowRight': 39
    0x87: 82,  # Run 'R' 82
    0x88: 40,  # 'ArrowDown': 40
    0x89: 83,  # Stop 'S' 83

    # type==3
    # button up
    0x01: 77,
    0x02: 38,
    0x03: 66,
    0x04: 37,
    0x05: 13,
    0x06: 39,
    0x07: 82,
    0x08: 40,
    0x09: 83,

    # type==2
    # short press
    0x11: 77,  # Home 'H' 72
    0x12: 38,  # 'ArrowUp': 38
    0x13: 66,  # Back 'B' 66
    0x14: 37,  # 'ArrowLeft': 37
    0x15: 13,  # 'Enter': 13
    0x16: 39,  # 'ArrowRight': 39
    0x17: 82,  # Run 'R' 82
    0x18: 40,  # 'ArrowDown': 40
    0x19: 83,  # Exit 'E' 69

    # type==4
    # long press
    0x91: 77,
    0x92: 38,
    0x93: 66,
    0x94: 37,
    0x95: 13,
    0x96: 39,
    0x97: 82,
    0x98: 40,
    0x99: 83,

    0xa9: 0x99,
    0x55: 0x98,
}

PowerMap = {
    0x1F: 100,
    0x0F: 75,
    0x07: 50,
    0x03: 25,
    0x01: 0
}


# 4.20V~3.90V满格
# 3.90V~3.80V三格
# 3.80V~3.72V两格
# 3.72V~3.65V一格
# 3.65以下，低电压告警。


'''
写地址0x46 设置打开的传感器及数据更新速率
0x10    1hz
0x20    10hz
0x30    50hz
0x40    100hz
0x50    1000hz

bit0,bit1,bit2分别是加速度、陀螺仪、地磁的开关，写入1打开，0关闭   
0x00    关7闭三者        
0x07    打开三者            
0x04    打开加速度           
0x02    打开陀螺仪           
0x01    打开地磁            
0x06    打开加速度和陀螺仪           
0x05    打开加速度和地磁            
0x03    打开陀螺仪和地磁            
'''


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
        self.m031_addr = 0x15
        self.int_pin = 22  # GPIO25 40pin 第22号引脚
        self.bus = smbus.SMBus(1)
        if btn_handler is not None:
            self.btn_handler = btn_handler
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.int_pin, GPIO.IN)
            GPIO.add_event_detect(self.int_pin, GPIO.BOTH,
                                callback=self.int_handler, bouncetime=20)

    def nineAxisSetEnable(self, value=0x47):
        """
        使能九轴传感器
        """
        self.bus.write_byte_data(self.m031_addr, 0x46, value)

    def int_handler(self, channel):
        """
        中断处理函数(按键)
        """
        btn = self.bus.read_byte_data(self.m031_addr, ReadButtonState)
        if self.btn_handler != None:
            if self.btn_handler(btn):
                return
        print(btn)
        if Button.has_key(btn):
            print(Button[btn])

    def readAccData(self):
        """
        读取加速度传感器
        Returns:
        x、y、z轴数据
        """
        data = self.bus.read_i2c_block_data(self.m031_addr, ReadAccSensor, 6)
        acc_x = data[1] << 8 | data[0]
        acc_y = data[3] << 8 | data[2]
        acc_z = data[5] << 8 | data[4]
        print("Acc_X=: "+str(acc_x)+"    Acc_Y=: "+str(acc_y)+"   Acc_Z: "+str(acc_z))
        return (acc_x, acc_y, acc_z)

    def readGyroData(self):
        """
        读取陀螺仪传感器
        Returns:
        x、y、z轴数据
        """
        data = self.bus.read_i2c_block_data(self.m031_addr, ReadGyroSensor, 6)
        gyro_x = data[1] << 8 | data[0]
        gyro_y = data[3] << 8 | data[2]
        gyro_z = data[5] << 8 | data[4]
        print("Gyro_X=: "+str(gyro_x)+"    Gyro_Y=: "+str(gyro_y)+"   Gyro_Z: "+str(gyro_z))
        return (gyro_x, gyro_y, gyro_z)

    def readMagnData(self):
        """
        读取地磁传感器
        Returns:
        x、y、z轴数据
        """
        data = self.bus.read_i2c_block_data(
            self.m031_addr, ReadMagnetometer, 6)
        magn_x = data[1] << 8 | data[0]
        magn_y = data[3] << 8 | data[2]
        magn_z = data[5] << 8 | data[4]
        print("Magn_X=: "+str(magn_x)+"    Magn_Y=: "+str(magn_y)+"   Magn_Z: "+str(magn_z))
        return (magn_x, magn_y, magn_z)

    def readBatOcv(self):
        """
        读取电池信息
        Returns:
        charging: 充电状态
        bat_power_ocv: 电池开路电压
        n_power: 电量指示灯
        est_power: 电量估计值(不太准确)
        """
        power = self.bus.read_i2c_block_data(self.m031_addr, ReadBatOcv, 4)
        # print(power[0])
        n_power = 0
        if PowerMap.has_key(power[0]):
            n_power = PowerMap[power[0]]
        # print("剩余电量: "+str(n_power))
        charging = power[1] & 0xF0
        bat_power_ocv = (power[3] << 8 | power[2])*0.26855/1000+2.6
        est_power = int( (bat_power_ocv-3.625)*200)
        # est_power = power[0]
        if est_power > 100:
           est_power = 100
        elif est_power < 0:
           est_power = 0
        if(charging):
            est_powerc=est_power&0x1F
            if(est_powerc==0x1F):
                est_power=100
            elif(est_powerc==0x0F):
                est_power=75
            elif(est_powerc==0x07):
                est_power=50
            elif(est_powerc==0x03):
                est_power=25
            elif(est_powerc==0x01):
                est_power==1
            else:
                #est_power=99;
                pass
    # print("电池开路电压=:"+str(bat_power_ocv)+"v")
        return (charging, bat_power_ocv, n_power, est_power)

    def readVout(self):
        vout12A = self.bus.read_i2c_block_data(self.m031_addr, 0x8B, 4)
        vout1A = (vout12A[1] << 8 | vout12A[0])*0.6394
        vout2A = (vout12A[3] << 8 | vout12A[2])*0.6394
        print("vout1A:"+str(vout1A)+"mA     vout2A:"+str(vout2A)+"mA")

    def readBat(self):
        vout12A = self.bus.read_i2c_block_data(self.m031_addr, 0x8B, 4)
        BAT = self.bus.read_i2c_block_data(self.m031_addr, 0x8C, 4)
        BAT_V = (vout12A[1] << 8 | vout12A[0])*0.26855/1000+2.6
        BAT_A = (vout12A[3] << 8 | vout12A[2])*1.27883/1000
        print("电池电压:"+str(BAT_V)+"V     电池电流:"+str(BAT_A)+"A")


if __name__ == '__main__':
    def test_print(data):
        print('read value %d ' % (data))
        return False
    driver = I2cDriver(test_print)
    while True:
        print('acc:', driver.readAccSensor())
        print('gyro:', driver.readGyroSensor())
        print('magn:', driver.readMagnSensor())
        print('power:', driver.readBatOcv())
        driver.readVout()
        driver.readBat()
        time.sleep(1)

# while(1):
#     print(bus.read_i2c_block_data(adress,0x83,6))
#     #time.sleep(1)
#     print(bus.read_i2c_block_data(adress,0x84,6))
#     #time.sleep(1)
#     print(bus.read_i2c_block_data(adress,0x85,6))
#     time.sleep(1)
#     print(bus.read_i2c_block_data(adress,0x8A,2))
#     #time.sleep(1)
#     print(bus.read_i2c_block_data(adress,0x8B,4))
#     print(bus.read_byte_data(adress,0x8C))

