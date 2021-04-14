#!coding:utf-8
from collections import deque
import math
import numpy as np

from kinematics import path_rotate_z

g_steps = 20
g_radius = 40

pi = math.acos(-1)


def any_direction_generator(radius, steps, direction=0, reverse=False):
    assert (steps % 4) == 0
    halfsteps = int(steps/2)
    rad = direction/180.0*math.pi
    step_angle = pi / halfsteps

    result = []

    # first half, move backward (only y change)
    for i in range(halfsteps):
        result.append((0, radius - i*radius*2/(halfsteps), 0))

    # second half, move forward in semicircle shape (y, z change)
    for i in range(halfsteps):
        angle = pi - step_angle*i
        y = radius * math.cos(angle)
        z = radius * math.sin(angle)
        result.append((0, y, z))

    for i in range(len(result)):
        x, y, z = result[i]
        result[i] = [x*math.cos(rad) - y*math.sin(rad),
                     x*math.sin(rad) + y*math.cos(rad), z]

    result = deque(result)
    # 从半圆的圆心开始运动,90度
    result.rotate(-int(steps/4))
    if reverse:
        result = deque(reversed(result))
    # print('result', result)
    return result


def path_generator(direction=0):
    assert (g_steps % 4) == 0
    halfsteps = int(g_steps/2)

    path = any_direction_generator(
        g_radius, g_steps, direction=direction, reverse=True)
    # print('path', path)
    mir_path = deque(path)
    # print('mir_path', mir_path)
    mir_path.rotate(halfsteps)
    # print('mir_path.rotate', mir_path)
    return [path, mir_path, path, mir_path, path, mir_path, ]


def turn_left_generator(direction=0):
    assert (g_steps % 4) == 0
    halfsteps = int(g_steps/2)

    path = any_direction_generator(
        g_radius, g_steps, direction=direction, reverse=False)
    mir_path = deque(path)
    mir_path.rotate(halfsteps)
    return [path_rotate_z(path, 45),
            path_rotate_z(mir_path, 0),
            path_rotate_z(path, 315),
            path_rotate_z(mir_path, 225),
            path_rotate_z(path, 180),
            path_rotate_z(mir_path, 135), ]


def turn_right_generator(direction=180):
    assert (g_steps % 4) == 0
    halfsteps = int(g_steps/2)

    path = any_direction_generator(
        g_radius, g_steps, direction=direction, reverse=False)
    mir_path = deque(path)
    mir_path.rotate(halfsteps)
    return [path_rotate_z(path, 45),
            path_rotate_z(mir_path, 0),
            path_rotate_z(path, 315),
            path_rotate_z(mir_path, 225),
            path_rotate_z(path, 180),
            path_rotate_z(mir_path, 135), ]


if __name__ == '__main__':
    from kinematics import tip_to_leg, ik, leg_ki
    path = path_generator()
    seq = path[0]
    # seq = [[0, 0, 0]]
    for point in seq:
        leg = tip_to_leg(0, point)
        angle = ik(leg)
        print('leg', leg)
        print('angle', angle)
        print('position', leg_ki(0, angle))
