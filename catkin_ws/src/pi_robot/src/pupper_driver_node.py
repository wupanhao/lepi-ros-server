#!/usr/bin/python
import threading
import numpy as np
import time
# from src.IMU import IMU
from pupper_controller import Controller
# from pupper_controller import JoystickInterface
from pupper_controller import State, Command, BehaviorState
from pupper_controller import HardwareInterface
from pupper_controller import Configuration
from pupper_controller import four_legs_inverse_kinematics
from pupper_controller import bainian

import math
import rospy
import json
from pi_driver.srv import SetString, SetStringResponse, GetString, GetStringResponse


BehaviorStateMap = {
    -1: BehaviorState.DEACTIVATED,
    0: BehaviorState.REST,
    1: BehaviorState.TROT,
    2: BehaviorState.HOP,
    3: BehaviorState.FINISHHOP,
    4: BehaviorState.BAINIAN,
    5: BehaviorState.FREE,
}


def limit_to(value, max_value):
    if value > max_value:
        value = max_value
    elif value < -max_value:
        value = -max_value
    return value


def toConfig(angles):
    states = np.zeros((3, 4))
    for i in range(4):
        for j in range(3):
            states[j][i] = angles[i*3+j]
    return states


class PupperDriverNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.active = True
        self.config = Configuration()
        self.hardware_interface = HardwareInterface()
        self.controller = Controller(
            self.config,
            four_legs_inverse_kinematics,
        )
        self.state = State()
        # self.joystick_interface = JoystickInterface(self.config)
        self.command = Command()
        reader = threading.Thread(target=self.loop)
        # reader.daemon = True
        reader.start()

        rospy.Service('~set_command', SetString, self.cbSetCommand)
        rospy.Service('~get_state', GetString, self.cbGetState)
        rospy.Service('~set_servo_angles', SetString, self.cbSetServoAngles)
        rospy.Service('~set_foot_locations', SetString,
                      self.cbSetFootLocations)
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

            if self.state.behavior_state == BehaviorState.BAINIAN:
                self.state.behavior_state = BehaviorState.REST
                for angles in bainian:
                    joint_angles = toConfig(angles)/180.0*math.pi
                    self.hardware_interface.set_actuator_postions(
                        joint_angles)
                    time.sleep(0.5)

            if self.state.behavior_state == BehaviorState.FREE:
                time.sleep(0.5)
                continue
            # quat_orientation = np.array([1, 0, 0, 0])
            # self.state.quat_orientation = quat_orientation
            self.controller.run(self.state, command)
            self.hardware_interface.set_actuator_postions(
                self.state.joint_angles)
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
                if data['height'] < 0.08:
                    data['height'] = 0.08
                if data['height'] > 0.22:
                    data['height'] = 0.22
                self.command.height = -data['height']
            if 'behavior_state' in data and data['behavior_state'] in BehaviorStateMap:
                self.state.behavior_state = BehaviorStateMap[data['behavior_state']]
            if 'servo_speed' in data and data['servo_speed'] > 0 and data['servo_speed'] <= 2000:
                self.hardware_interface.speed = int(data['servo_speed'])
        except Exception as e:
            print(e)
        return SetStringResponse(params.data)

    def get_command(self):
        return self.command

    def cbGetState(self, params):
        state = {
            "servo_angles": self.hardware_interface.servo_angles,
            "foot_locations": self.state.foot_locations.tolist()
        }
        return GetStringResponse(json.dumps(state))

    def cbSetServoAngles(self, params):
        joint_angles = json.loads(params.data)
        self.hardware_interface.set_actuator_angles(
            joint_angles)
        return SetStringResponse()

    def cbSetFootLocations(self, params):
        foot_locations = np.array(json.loads(params.data))
        print(foot_locations)
        joint_states = self.controller.inverse_kinematics(
            foot_locations, self.controller.config
        )
        self.hardware_interface.set_actuator_postions(
            joint_states)
        return SetStringResponse()

    def onShutdown(self):
        self.active = False
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('pupper_driver_node', anonymous=False)
    node = PupperDriverNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
