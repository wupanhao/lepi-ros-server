#!coding:utf-8
import struct
import time

BUTTON = 129
AXIS = 130

EVENT_BUTTON = 1
EVENT_AXIS = 2


class MyJoy(object):
    """
    MyJoy 类, joystick 手柄驱动
    Attributes:
    active: bool 活动状态
    callback: function 回调函数，每当有新按键数据自动调用
    axes: dict 坐标轴状态
    buttons: dict 按键状态
    infile_path: str 设备文件路径
    """

    def __init__(self, callback=None):
        super(MyJoy, self).__init__()
        self.active = True
        self.callback = callback
        self.axes = {}
        self.buttons = {}
        self.infile_path = "/dev/input/js0"

    def start_listen_loop(self):
        """
        start_listen_loop 函数, 按键监听循环
        """
        EVENT_SIZE = struct.calcsize("LhBB")
        file = open(self.infile_path, "rb")
        event = file.read(EVENT_SIZE)
        while event:
            # print(struct.unpack("LhBB", event))
            (tv_msec,  value, event_type,
                number) = struct.unpack("LhBB", event)
            # print(tv_msec,  value, event_type, number)
            if event_type == EVENT_BUTTON or event_type == BUTTON:
                self.buttons[number] = value
            elif event_type == EVENT_AXIS or event_type == AXIS:
                self.axes[number] = value
            print(self.getState())
            if self.callback is not None:
                self.callback()
            event = file.read(EVENT_SIZE)

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


if __name__ == '__main__':
    from car import CarDriver3
    import threading
    car = CarDriver3()
    joy = MyJoy(callback=None)
    reader = threading.Thread(target=joy.start_open_loop)
    reader.daemon = True
    reader.start()

    def setCarSpeed():
        if joy.axes.has_key(1) and joy.axes.has_key(3):
            speed = joy.axes[1]/32767.0
            steer = joy.axes[3]/32767.0
            car.setWheelsSpeed(speed*100, -steer*90)
    # joy = MyJoy(callback=setCarSpeed)
    while True:
        setCarSpeed()
        time.sleep(0.01)
    joy.active = False
