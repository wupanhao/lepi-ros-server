# coding:utf-8
import math

import config

pi = math.acos(-1)
defaultAngle = config.defaultAngle
mountPosition = config.mountPosition


def leg_ki(leg_index, angles):
    # 以body center为原点
    mount = mountPosition[leg_index]
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


defaultPosition = []
for i in range(6):
    leg = leg_ki(i, (0, 0, 0))
    defaultPosition.append(leg)
defaultPosition = config.defaultPosition


def point_rotate_x(pt, angle):
    rad = angle/180.0*math.pi
    y = pt[1]*math.cos(rad) - pt[2]*math.sin(rad)
    z = pt[1]*math.sin(rad) + pt[2]*math.cos(rad)
    x = pt[0]
    return [x, y, z]


def point_rotate_y(pt, angle):
    rad = angle/180.0*math.pi
    x = pt[0]*math.cos(rad) - pt[2]*math.sin(rad)
    z = pt[0]*math.sin(rad) + pt[2]*math.cos(rad)
    y = pt[1]
    return [x, y, z]


def point_rotate_z(pt, angle):
    # 绕z轴逆时针旋转
    # print(pt, angle)
    rad = angle/180.0*math.pi
    x = pt[0]*math.cos(rad) - pt[1]*math.sin(rad)
    y = pt[0]*math.sin(rad) + pt[1]*math.cos(rad)
    z = pt[2]
    return [x, y, z]


def path_rotate_z(path, angle):
    rotate_path = [point_rotate_z(pt, angle) for pt in path]
    return rotate_path


def point_rotate(pt, rotate=[0, 0, 0]):
    x, y, z = rotate
    if x % 360 != 0:
        pt = point_rotate_x(pt, x)
    if y % 360 != 0:
        pt = point_rotate_y(pt, y)
    if z % 360 != 0:
        pt = point_rotate_z(pt, z)
    return pt


def to_angle(rad):
    return rad/math.pi*180


def ik(to):
    angles = []
    x = to[0]
    y = to[1]
    # 首先要进行坐标变换改变参考系得到相对坐标
    # 以mountPoint即J1为坐标原点，初始角度为x轴，投影到xOy平面上计算第一个角
    angles.append(-math.atan2(y, x) * 180 / pi)

    # 以J2为坐标原点，把j2-j3-tip 这个三角形投影到xOz平面计算第二、三角
    x = math.sqrt(x*x + y*y) - config.kLegJ1ToJ2
    y = to[2]
    # j2看tip的仰角
    ar = math.atan2(y, x)
    lr2 = x*x + y*y
    lr = math.sqrt(lr2)
    # 余弦定理,j2-j3-tip 这个三角形
    # 角j3-j2-tip
    a1 = math.acos((lr2 + config.kLegJ2ToJ3*config.kLegJ2ToJ3 -
                    config.kLegJ3ToTip*config.kLegJ3ToTip)/(2*config.kLegJ2ToJ3*lr))
    # 角j3-tip-j2
    a2 = math.acos((lr2 - config.kLegJ2ToJ3*config.kLegJ2ToJ3 +
                    config.kLegJ3ToTip*config.kLegJ3ToTip)/(2*config.kLegJ3ToTip*lr))

    angles.append((ar + a1) * 180 / pi)
    angles.append(90 - ((a1 + a2) * 180 / pi))
    # print('ar', to_angle(ar), 'a1', to_angle(a1), 'a2', to_angle(a2))
    return angles


def ki(angles):
    # 以mountPoint即J1为坐标原点，初始角度为x轴
    l = config.kLegJ1ToJ2+config.kLegJ2ToJ3 * \
        math.cos(angles[1]/180.0*math.pi) + config.kLegJ3ToTip * \
        math.sin(angles[2]/180.0*math.pi)
    x = math.cos((angles[0])/180.0*math.pi) * l
    y = -math.sin((angles[0])/180.0*math.pi) * l
    z = config.kLegJ2ToJ3 * \
        math.sin(angles[1]/180.0*math.pi) - config.kLegJ3ToTip * \
        math.cos(angles[2]/180.0*math.pi)
    return [x, y, z]


# global: body center为原点，横纵为xy轴
# mount: mount为原点，坐标轴不变
# leg: mount为原点，leg初始角度为x轴
# tip: tip为原点，坐标轴不变
# path: tip坐标构成的序列

def global_to_mount(leg_index, co):
    # 以j1为原点的坐标，坐标轴不变
    mount = mountPosition[leg_index]
    x = co[0]-mount[0]
    y = co[1]-mount[1]
    z = co[2]-mount[2]
    return [x, y, z]


def to_leg_rotate(leg_index, co):
    # 以j1为原点的坐标，初始角度为x轴
    angle = defaultAngle[leg_index]
    pt = global_to_mount(leg_index, co)
    return point_rotate_z(pt, angle)


def tip_to_leg(leg_index, co):
    angle = defaultAngle[leg_index]
    position = defaultPosition[leg_index]
    pt = [position[0]+co[0], position[1]+co[1], position[2]+co[2]]
    # print('target', pt)
    pt = global_to_mount(leg_index, pt)
    # print('mount', pt)
    return point_rotate_z(pt, angle)


def global_to_leg(leg_index, pt):
    angle = defaultAngle[leg_index]
    pt = global_to_mount(leg_index, pt)
    # print('mount', pt)
    return point_rotate_z(pt, angle)


def mount_to_global(leg_index, co):
    # 以body center为原点的坐标
    mount = mountPosition[leg_index]
    x = [co[0]+mount[0]]
    y = [co[1]+mount[1]]
    z = [co[2]+mount[2]]
    return [x, y, z]


def tips_to_angles(tips):
    angles = []
    for j in range(6):
        leg = tip_to_leg(j, tips[j])
        # print(j, leg)
        angle = ik(leg)
        angles.extend(angle)
    return angles


def path_to_angles(path):
    steps = len(path[0])
    seqs = []
    for i in range(steps):
        # print(i, path)
        angles = tips_to_angles([path[j][i] for j in range(6)])
        seqs.append(angles)
    return seqs


if __name__ == '__main__':
    print('0 0 0')
    for i in range(6):
        print(i)
        pt = leg_ki(i, [0, 0, 0])
        print('point', pt)
        leg = to_leg_rotate(i, pt)
        angle = ik(leg)
        print('angle', angle)
    # angle = [9, 33, 50]
    # print(angle)
    # for i in range(6):
    #     pt = leg_ki(i, angle)
    #     print(i)
    #     print('point', pt)
    #     leg = to_leg_rotate(i, pt)
    #     print('leg', leg)
    #     angle = ik(leg)
    #     print('angle', angle)
