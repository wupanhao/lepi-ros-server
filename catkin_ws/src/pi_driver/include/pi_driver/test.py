import spidev
import time
import ctypes

LP_SPI = spidev.SpiDev()
LP_SPI.open(0, 1)
LP_SPI.max_speed_hz = 50000
LP_SPI.mode = 0b00
LP_SPI.bits_per_word = 8

print(LP_SPI.xfer2([1,2,3,4,5,6,7,8,9,10]))
