#!/usr/bin/python
import threading
import numpy as np
import time

# from hexapod_controller import JoystickInterface
from hexapod_controller import HardwareInterface
from hexapod_controller import NEUTRAL_ANGLE_DEGREES

from hexapod_controller import Solver
from hexapod_controller import Controller
from hexapod_controller import Configuration, State, BehaviorState, Command

import math
import rospy
import json
from pi_driver.srv import SetString, SetStringResponse


BehaviorStateMap = {
    -1: BehaviorState.DEACTIVATED,
    0: BehaviorState.REST,
    1: BehaviorState.TROT,
    2: BehaviorState.HOP,
    3: BehaviorState.FINISHHOP,
}


def limit_to(value, max_value):
    if value > max_value:
        value = max_value
    elif value < -max_value:
        value = -max_value
    return value


class HexapodDriverNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.active = True
        self.config = Configuration()
        solver = Solver(self.config)
        self.hardware_interface = HardwareInterface(NEUTRAL_ANGLE_DEGREES)
        self.controller = Controller(
            self.config,
            solver.inverse_kinematics_body,
        )
        self.state = State()
        # self.joystick_interface = JoystickInterface(self.config)
        self.command = Command()
        reader = threading.Thread(target=self.loop)
        # reader.daemon = True
        reader.start()

        rospy.Service('~set_command', SetString, self.cbSetCommand)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def loop(self):
        print('start loop')
        last_loop = time.time()
        # Wait until the activate button has been pressed
        while self.active:
            now = time.time()
            if now - last_loop < self.config.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = self.get_command()

            # quat_orientation = np.array([1, 0, 0, 0])
            # self.state.quat_orientation = quat_orientation
            try:
                self.controller.run(self.state, command)
                self.hardware_interface.set_actuator_postions(
                    self.state.joint_angles)
            except Exception as e:
                print(e)
        print('end loop')

    def cbSetCommand(self, params):
        try:
            data = json.loads(params.data)
            print(data)
            if 'horizontal_velocity_x' in data:
                value = limit_to(data['horizontal_velocity_x'],
                                 self.config.max_x_velocity)
                self.command.horizontal_velocity[0] = value
            if 'horizontal_velocity_y' in data:
                value = limit_to(data['horizontal_velocity_y'],
                                 self.config.max_y_velocity)
                self.command.horizontal_velocity[1] = value
            if 'roll' in data:
                self.command.roll = limit_to(
                    data['roll'], self.config.max_roll)
            if 'pitch' in data:
                self.command.pitch = limit_to(
                    data['pitch'], self.config.max_pitch)
            if 'yaw_rate' in data:
                self.command.yaw_rate = limit_to(
                    data['yaw_rate'], self.config.max_yaw_rate)
            if 'height' in data:
                if data['height'] < -0.08:
                    data['height'] = -0.08
                if data['height'] > 0.17:
                    data['height'] = 0.17
                self.command.height = -data['height']
            if 'behavior_state' in data and data['behavior_state'] in BehaviorStateMap:
                self.state.behavior_state = BehaviorStateMap[data['behavior_state']]
        except Exception as e:
            print(e)
        return SetStringResponse(params.data)

    def get_command(self):
        return self.command

    def onShutdown(self):
        self.active = False
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('hexapod_driver_node', anonymous=False)
    node = HexapodDriverNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
