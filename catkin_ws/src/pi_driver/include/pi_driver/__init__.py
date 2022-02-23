try:
    with open("/proc/cpuinfo", 'r') as f:
        lines = f.readlines()
        #print(lines[-1],"Pi 3 Model B" in lines[-1])
        if "Pi 3 Model B" in lines[-1]:
            # print('pi3')
            from .pi3_driver import *
        else:
            # print('pi4')
            from .pi4_driver import *
except Exception as e:
    print(e)

from .button_subscriber import *
from .button_subscriber2 import *
from .joystick2 import MyJoy
import sys
sys.path.append('.')
from . import vcgencmd
