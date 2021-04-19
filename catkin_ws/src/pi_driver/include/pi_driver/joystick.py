#!coding:utf-8
import struct
import time
# import thread
import threading
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
        # thread.start_new_thread(self.start_open_loop, ())
        reader = threading.Thread(target=self.start_open_loop)
        # reader.daemon = True
        reader.start()

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
            if self.callback is not None:
                self.callback()
            # print(self.getState())
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
    import threading
    joy = MyJoy(callback=None)
    # reader = threading.Thread(target=joy.start_open_loop)
    # reader.daemon = True
    # reader.start()

    time.sleep(10)
