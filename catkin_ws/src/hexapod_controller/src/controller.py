#!/usr/bin/python
import math
import time
import yaml
import os
import json
import config
from kinematics import tip_to_leg, ik, leg_ki, point_rotate, global_to_leg, path_to_angles
from movement import path_generator, turn_left_generator, turn_right_generator

defaultAngle = config.defaultAngle
mountPosition = config.mountPosition


def point_add(p1, p2):
    return [p1[0]+p2[0], p1[1]+p2[1], p1[2]+p2[2]]


class HexapodController:
    def __init__(self):
        self.startAngle = (0, 30, -15)
        # body center即原点坐标
        self.position = [0, 0, 0]
        self.rotation = [0, 0, 0]

        self.mountPosition = self.computeMountPosition()
        self.tipPosition = self.computeTipPosition()

    def computeMountPosition(self):
        result = []
        for i in range(6):
            result.append(point_add(self.position, mountPosition[i]))
        return result

    def computeTipPosition(self):
        result = []
        for i in range(6):
            self.leg_ki(i, self.startAngle)
        return result

    def leg_ki(self, leg_index, angles):
        # 以body center为原点
        mount = self.mountPosition[leg_index]
        l = config.kLegJ1ToJ2+config.kLegJ2ToJ3 * \
            math.cos(angles[1]/180.0*math.pi) + config.kLegJ3ToTip * \
            math.sin((angles[1]+angles[2])/180.0*math.pi)
        x = math.cos((defaultAngle[leg_index]+angles[0])/180.0*math.pi) * l
        y = -math.sin((defaultAngle[leg_index]+angles[0])/180.0*math.pi) * l
        z = config.kLegJ2ToJ3 * \
            math.sin(angles[1]/180.0*math.pi) - config.kLegJ3ToTip * \
            math.cos((angles[1]+angles[2])/180.0*math.pi)
        # return [x, y, z]
        return [mount[0] + x, mount[1] + y, mount[2]+z]

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

    def static_position(self, position=[0, 0, 0]):
        angles = []
        for j in range(6):
            leg = tip_to_leg(j, position)
            angle = ik(leg)
            angles.extend(angle)
            # print(j, leg, angle)

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


if __name__ == '__main__':
    controller = HexapodController()
