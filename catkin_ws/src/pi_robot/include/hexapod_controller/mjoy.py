#!coding:utf-8
import evdev
from evdev import InputDevice, categorize, ecodes
import threading
import math
import time

KeyCodeMap = {
    ecodes.ABS_X: 0,
    ecodes.ABS_Y: 1,
    ecodes.ABS_Z: 2,
    ecodes.ABS_RX: 3,
    ecodes.ABS_RY: 4,
    ecodes.ABS_RZ: 5,
    ecodes.BTN_SOUTH: 0,
    ecodes.BTN_EAST: 1,
    ecodes.BTN_NORTH: 2,
    ecodes.BTN_WEST: 3,
    ecodes.BTN_TL: 4,
    ecodes.BTN_TR: 5,
    ecodes.BTN_TL2: 6,
    ecodes.BTN_TR2: 7,
    ecodes.BTN_SELECT: 8,
    ecodes.BTN_START: 9,
    ecodes.BTN_MODE: 10,
    ecodes.BTN_THUMBL: 11,
    ecodes.BTN_THUMBR: 12,
    ecodes.BTN_DPAD_UP: 13,
    ecodes.BTN_DPAD_DOWN: 14,
    ecodes.BTN_DPAD_LEFT: 15,
    ecodes.BTN_DPAD_RIGHT: 16,
}


class MyJoy:
    def __init__(self, callback=None):
        self.active = True
        self.axes = {}
        for i in range(8):
            self.axes[i] = 0
        self.buttons = {}
        for i in range(17):
            self.buttons[i] = 0
        self.callback = callback
        reader = threading.Thread(target=self.start_open_loop)
        reader.daemon = True
        reader.start()

    def start_listen_loop(self):
        """
        start_listen_loop 函数, 按键监听循环
        """
        self.dev = None
        for path in evdev.list_devices():
            device = evdev.InputDevice(path)
            # device /dev/input/event1, name "Sony PLAYSTATION(R)3 Controller", phys "e4:5f:01:5e:52:10"
            print(device.path, device.name, device.phys, device.capabilities())
            # if device.name == 'Sony PLAYSTATION(R)3 Controller':
            if ecodes.EV_FF in device.capabilities():
                self.dev = InputDevice(device.path)
                break
            else:
                device.close()
        print(self.dev)
        if self.dev is None:
            print('device not find')
            return
        for event in self.dev.read_loop():
            # 按键
            if event.type == ecodes.EV_KEY:
                if event.code in KeyCodeMap:
                    self.buttons[KeyCodeMap[event.code]] = event.value
                if event.value != 2 and self.callback is not None:
                    self.callback(event)
                elif self.callback is None:
                    pass
                    # print(event)
                    # print(categorize(event))
            # 摇杆
            elif event.type == ecodes.EV_ABS:
                if event.code in KeyCodeMap:
                    self.axes[KeyCodeMap[event.code]] = math.ceil(
                        (event.value - 128)/128.0*100)
                # print(event)
                # print(categorize(event))

    def start_open_loop(self):
        """
        start_open_loop 函数, 设备打开循环
        """
        while self.active:
            try:
                self.start_listen_loop()
            except Exception as e:
                print(e)
                print('Read %s Error, retry after 2 seconds', self.infile_path)
            time.sleep(2)

    def getState(self):
        """
        getState 函数, 返回手柄状态
        """
        state = {'Axes': self.axes, 'Buttons': self.buttons}
        return state

    def getAxisValue(self, id):
        time.sleep(0.005)
        if id in self.axes:
            return self.axes[id]
        else:
            return 0

    def getButtonState(self, id):
        time.sleep(0.005)
        if id in self.buttons:
            return self.buttons[id]
        else:
            return 0


if __name__ == '__main__':
    import time
    mjoy = MyJoy()
    time.sleep(1000)
