#coding: utf-8
import os
from pi_driver.msg import ButtonEvent
from threading import Thread
import rospy


class KEY:
    ArrowLeft = 37
    ArrowUp = 38
    ArrowRight = 39
    ArrowDown = 40
    Enter = 13
    M = 77  # KeyM (Menu)
    B = 66  # KeyB (Back)
    R = 82  # KeyR (Run)
    S = 83  # KeyS (Stop or Home)


class RosButtonSubscriber:
    def __init__(self, callback=None):
        # 忽略用户输入,打开可用os.system("stty echo")
        # 清除忽略的用户输入,termios.tcflush(sys.stdin, termios.TCIOFLUSH)
        os.system("stty -echo")
        button_topic = '/ubiquityrobot/pi_driver_node/button_event'
        self.callback = callback
        self.buttonState = {
            KEY.ArrowLeft: 0,
            KEY.ArrowUp: 0,
            KEY.ArrowRight: 0,
            KEY.ArrowDown: 0,
            KEY.Enter: 0,
            KEY.M: 0,  # KeyM (Menu)
            KEY.B: 0,  # KeyB (Back)
            KEY.R: 0,  # KeyR (Run)
            KEY.S: 0  # KeyS (Stop or Home)
        }
        rospy.init_node('button_subscriber', anonymous=True)
        rospy.Subscriber(button_topic, ButtonEvent,
                         self.cbButton, queue_size=1)

    def cbButton(self, button_msg):
        self.buttonState[button_msg.value] = button_msg.type
        if self.callback is None:
            print(button_msg)
        else:
            self.callback(button_msg)

    def keyPressed(self, key):
        if self.buttonState[key] == 1 or self.buttonState[key] == 4:
            return True
        else:
            return False


threads = {}
onKeyPressed = {
    KEY.ArrowLeft: [],
    KEY.ArrowUp: [],
    KEY.ArrowRight: [],
    KEY.ArrowDown: [],
    KEY.Enter: [],
    KEY.M: [],  # KeyM (Menu)
    KEY.B: [],  # KeyB (Back)
    KEY.R: [],  # KeyR (Run)
    KEY.S: []  # KeyS (Stop or Home)
}


def buttonHandler(button_msg):
    global threads
    global onKeyPressed
    if button_msg.type != 1:
        return

    if button_msg.value in onKeyPressed and len(onKeyPressed[button_msg.value]) > 0:
        for i, callback in enumerate(onKeyPressed[button_msg.value]):
            funcName = '%d-%d' % (button_msg.value, i)
            if funcName in threads and threads[funcName].is_alive():
                print('模块 ' + funcName + ' 正在运行')
                continue
            else:
                threads[funcName] = Thread(target=callback)
                threads[funcName].start()
    else:
        return
        print('没有为该按键定义处理函数:%d' % (button_msg.value))


if __name__ == '__main__':
    button = RosButtonSubscriber()
    rospy.spin()
