#!/usr/bin/python
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from pi_driver import SServo, Servo
from hexapod_controller.srv import GetOffsets, GetOffsetsResponse, SetOffsets, SetOffsetsResponse
import time
import yaml
import os
import threading
import json
import config
from kinematics import tip_to_leg, ik, leg_ki, point_rotate, global_to_leg, path_to_angles
from movement import path_generator, turn_left_generator, turn_right_generator


class HexapodControllerNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.alive = True
        self.count = 0
        self.center = 1023/2
        self.speed = 1000
        self.joyState = None
        self.servoState = None
        self.mode = 'move'
        self.move_dir = None
        self.move_speed = 0
        self.steps = None
        self.cali_file = os.path.expanduser(
            '~') + '/Lepi_Data/ros/hexapod_controller/calibrations.yaml'
        self.pub_node_state = rospy.Publisher(
            "/node_state", String, queue_size=1)
        self.pub_node_state.publish(
            String('{"node":"%s","state":"%s"}' % (self.node_name, "started")))
        self.loadCaliFile()
        self.attatchServos()
        # self.toCenter()
        self.sub_joy_state = rospy.Subscriber(
            "/ubiquityrobot/joystick_node/joy_state", String, self.cbJoyState, queue_size=1)
        self.sub_joint_state = rospy.Subscriber(
            "~joint_states", JointState, self.cbJointState, queue_size=1)
        self.sub_joint_angle = rospy.Subscriber(
            "~joint_angles", JointState, self.cbJointAngle, queue_size=1)
        self.srv_get_offsets = rospy.Service(
            "~get_offsets", GetOffsets, self.loadCaliFile)
        self.srv_set_offsets = rospy.Service(
            "~set_offsets", SetOffsets, self.saveCaliFile)
        self.turn_steps = {
            "left": path_to_angles(turn_left_generator()),
            "right": path_to_angles(turn_right_generator()),
        }
        self.move_steps = []
        start = time.time()
        for i in range(360):
            self.move_steps.append(path_to_angles(path_generator(i)))
        end = time.time()
        print('generate move_path costs ,', end-start)
        self.loop = threading.Thread(target=self.start_move_loop)
        # reader.daemon = True
        self.loop.start()
        # for i in range(20):
        #     self.test_move(i*90 % 360)
        # self.test_position_xy()
        # self.test_rotate()
        # self.test_turn()
        rospy.loginfo("[%s] Initialized......" % (self.node_name))
        #os.system('bash -c "source /home/pi/nodejs.sh && node /home/pi/workspace/lepi-gui/app/hexapod/controller.js"')

    def attatchServos(self, port=None):
        try:
            self.servos = SServo(port)
        except Exception as e:
            self.servos = None
            print(e)
            exit()

    def toCenter(self):
        msg = JointState()
        # msg.position = self.offsets
        # self.cbJointAngle(msg)
        rads = [0 for i in range(18)]
        msg.position = rads
        self.cbJointState(msg)

    def cbServoAngles(self, angles):
        # for i in [2,5,8,11,14,17]:
        #    positions[i] = - positions[i]
        servo_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9,
                     16, 17, 18, 13, 14, 15, 10, 11, 12]
        array = [0 for i in range(18)]
        for i in range(18):
            array[i] = angles[i] + self.offsets[servo_ids[i]-1]
            if i % 3 == 2:
                array[i] = -array[i]
        if self.servoState is None:
            self.servoState = array
        if False:
            offset = [array[i] - self.servoState[i] for i in range(18)]
            print('offset', offset, 'max', max(offset), 'min', min(offset))
            n_step = int(max([abs(max(offset)), abs(min(offset))]) / 20.0)+1
            for j in range(n_step):
                array = [self.servoState[i] +
                         float(offset[i])/n_step for i in range(18)]
                positions = [int(array[i]/200.0*1023)
                             for i in range(len(array))]
                servos = [Servo(servo_ids[i], positions[i]+self.center, speed=self.speed)
                          for i in range(len(array))]
                if self.servos is not None:
                    self.servos.set_positions_sync(servos)
                if j < n_step - 1:
                    time.sleep(0.04)
        else:
            positions = [int(array[i]/200.0*1023)
                         for i in range(len(array))]
            servos = [Servo(servo_ids[i], positions[i]+self.center, speed=self.speed)
                      for i in range(len(array))]
            if self.servos is not None:
                self.servos.set_positions_sync(servos)

    def cbServoPositions(self, positions):
        #servo_ids = [7, 8, 9, 10, 11, 12, 13, 14, 15, 4, 5, 6, 1, 2, 3, 16, 17, 18]
        servos = [Servo(i+1, positions[i], speed=self.speed)
                  for i in range(len(positions))]
        if self.servos is not None:
            self.servos.set_positions_sync(servos)

    def cbJointState(self, msg):
        positions = msg.position
        # print(msg)
        angles = [int(positions[i]/math.pi*180)
                  for i in range(len(positions))]
        self.cbServoAngles(angles)

    def cbJointAngle(self, msg):
        angles = [int(i) for i in msg.position]
        self.cbServoAngles(angles)

    def loadCaliFile(self, params=None):
        res = GetOffsetsResponse()
        try:
            with open(self.cali_file, 'r') as stream:
                data = yaml.load(stream)
                self.offsets = data['offsets']
        except Exception as e:
            print(e)
            self.offsets = [0 for i in range(18)]
        res.data.position = self.offsets
        return res

    def saveCaliFile(self, params=None):
        self.offsets = list(params.data.position)
        res = SetOffsetsResponse()
        with open(self.cali_file, 'w') as stream:
            yaml.dump({"offsets": self.offsets}, stream)
        return res

    def onShutdown(self):
        self.alive = False
        self.pub_node_state.publish(
            String('{"node":"%s","state":"%s"}' % (self.node_name, "stopped")))
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

    def static_rotate(self, rotate=[0, 0, 0]):
        angles = []
        position = [0, 0, 0]
        position = config.defaultPosition
        for j in range(6):
            pt = position[j]
            pt = point_rotate(pt, rotate)
            leg = global_to_leg(j, pt)
            angle = ik(leg)
            angles.extend(angle)
            # print(j, leg, angle)
        self.cbServoAngles(angles)

    def static_position(self, position=[0, 0, 0]):
        angles = []
        for j in range(6):
            leg = tip_to_leg(j, position)
            angle = ik(leg)
            angles.extend(angle)
            # print(j, leg, angle)
        self.cbServoAngles(angles)

    def test_move(self, dir=0):
        path = path_generator(dir)
        seq = path_to_angles(path)
        for i in range(len(seq)):
            angles = seq[i]
            self.cbServoAngles(angles)
            time.sleep(0.04)

    def test_position_z(self):
        for i in range(0, 60):
            self.static_position([0, 0, -i])
            time.sleep(0.04)
        for i in range(0, 60):
            self.static_position([0, 0, -60+i])
            time.sleep(0.04)

    def test_position_xy(self):
        rad = 60  # 6cm
        for i in range(0, rad):
            self.static_position([i, 0, 0])
            time.sleep(0.03)
        for i in range(0, 360):
            self.static_position(
                [rad*math.cos(i/180.0*math.pi), rad*math.sin(i/180.0*math.pi), 0])
            time.sleep(0.04)
        for i in range(0, rad):
            self.static_position([rad-i, 0, 0])
            time.sleep(0.03)

    def test_rotate(self):
        for i in range(15):
            self.static_rotate([i, 0, 0])
            time.sleep(0.05)
        for i in range(15):
            self.static_rotate([15-i, 0, 0])
            time.sleep(0.05)
        for i in range(15):
            self.static_rotate([0,  i, 0])
            time.sleep(0.05)
        for i in range(15):
            self.static_rotate([0,  15-i, 0])
            time.sleep(0.05)
        for i in range(15):
            self.static_rotate([0, 0, i])
            time.sleep(0.05)
        for i in range(15):
            self.static_rotate([0, 0, 15-i])
            time.sleep(0.05)

    def test_turn(self):
        path = turn_left_generator()
        seq = path_to_angles(path)
        for i in range(len(seq)):
            angles = seq[i]
            self.cbServoAngles(angles)
            time.sleep(0.04)
        path = turn_right_generator()
        seq = path_to_angles(path)
        for i in range(len(seq)):
            angles = seq[i]
            self.cbServoAngles(angles)
            time.sleep(0.04)

    def cbJoyState(self, msg):
        # print(msg)
        joy = json.loads(msg.data)
        if self.joyState is None:
            self.joyState = joy
        # print(joy)

        # L1 = joy["Buttons"]["4"]
        R1 = joy["Buttons"]["5"]
        if R1 == 1 and self.joyState["Buttons"]["5"] == 0:
            # change mode
            if self.mode == 'move':
                self.mode = 'stand'
            else:
                self.steps = None
                self.mode = 'move'
            print(self.mode)

        left_x = joy["Axes"]["0"]
        left_y = -joy["Axes"]["1"]
        left_rad = math.atan2(left_y, left_x)
        left_dir = left_rad/math.pi*180
        right_x = joy["Axes"]["3"]
        right_y = -joy["Axes"]["4"]
        right_rad = math.atan2(right_y, right_x)
        right_dir = right_rad/math.pi*180
        self.joyState = joy

        if self.mode == 'move':
            if abs(right_x) > 1000:
                self.turn_dir = 'right' if right_x > 0 else 'left'
                self.move_speed = abs(right_x)
                self.steps = self.turn_steps[self.turn_dir]
            else:
                self.move_speed = abs(left_x) if abs(
                    left_x) > abs(left_y) else abs(left_y)
                move_dir = int(left_dir + 90)
                if move_dir < 0:
                    move_dir = move_dir+360
                if self.move_dir != move_dir:
                    self.move_dir = move_dir
                    # self.steps = path_to_angles(path_generator(move_dir))
                    self.steps = self.move_steps[move_dir]

        elif self.mode == 'stand':
            left_speed = abs(left_x) if abs(
                left_x) > abs(left_y) else abs(left_y)
            right_speed = abs(right_x) if abs(
                right_x) > abs(right_y) else abs(right_y)
            if right_speed > 1000:
                rotate_y = 10 * math.cos(right_rad) * right_speed/32767.0
                rotate_x = 10 * math.sin(right_rad)*right_speed/32767.0
                self.static_rotate([rotate_x, rotate_y, 0])
            elif left_speed > 1000:
                translate_x = 35*math.cos(left_rad)*left_speed/32767.0
                translate_y = 35*math.sin(left_rad)*left_speed/32767.0
                self.static_position([-translate_x, -translate_y, 0])
        # print('left_dir', left_dir, 'right_dir', right_dir)

    def start_move_loop(self):
        self.static_position()
        # for i in range(4):
        #     self.test_move(i*90)
        i = 0
        while self.alive:
            if self.steps is None:
                time.sleep(0.05)
                continue
            while self.mode == 'move':
                if self.move_speed > 1000:
                    if i >= len(self.steps):
                        i = 0
                    angles = self.steps[i]
                    self.cbServoAngles(angles)
                    i = i + 1
                    time.sleep(1000.0/(self.move_speed+1000))
                else:
                    time.sleep(0.05)
                    break


if __name__ == '__main__':
    rospy.init_node('hexapod_controller_node', anonymous=False)
    node = HexapodControllerNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
