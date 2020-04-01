#!/usr/bin/python

import rospy
import thread
from pi_driver import MyJoy
from pi_driver.srv import GetString, GetInt32, GetStringResponse, GetInt32Response
from std_msgs.msg import String
import json


class JoystickNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.joy = MyJoy()
        self.json_state = json.dumps(self.joy.getState())
        self.srv_get_button_value = rospy.Service(
            '~get_button_value', GetInt32, self.cbGetButtonValue)
        self.srv_get_axis_value = rospy.Service(
            '~motor_get_position', GetInt32, self.cbGetAxisValue)
        self.srv_get_joy_state = rospy.Service(
            "~get_joy_state", GetString, self.cbGetJoyState)
        self.pub_joy_state = rospy.Publisher(
            "~joy_state", String, queue_size=1)
        rospy.Timer(rospy.Duration.from_sec(1.0/30), self.cbPublishJoyState)
        rospy.loginfo("[%s] Initialized......" % (self.node_name))

    def cbGetButtonValue(self, msg):
        if self.joy.buttons.has_key(msg.port):
            return GetInt32Response(self.joy.buttons[msg.port])
        else:
            return GetInt32Response()

    def cbGetAxisValue(self, msg):
        if self.joy.axes.has_key(msg.port):
            return GetInt32Response(self.joy.axes[msg.port])
        else:
            return GetInt32Response()

    def cbGetJoyState(self, msg):
        return GetStringResponse(self.json_state)

    def cbPublishJoyState(self, channel):
        self.json_state = json.dumps(self.joy.getState())
        print(self.json_state)
        self.pub_joy_state.publish(String(self.json_state))

    def onShutdown(self):
        self.joy.active = False
        rospy.loginfo("[%s] shutdown......" % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('joystick_node', anonymous=False)
    node = JoystickNode()
    rospy.on_shutdown(node.onShutdown)
    thread.start_new_thread(node.joy.start_open_loop, ())
    # node.joy.start_open_loop()
    rospy.spin()
