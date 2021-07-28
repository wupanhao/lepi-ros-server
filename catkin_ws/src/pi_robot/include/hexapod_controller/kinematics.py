#!coding:utf-8
import math
import numpy as np
# defaultAngle = (
#     -45, 0, 45, 135, 180, 225
# )

defaultAngle = (
    45, 0, -45, 225, 180, 135
)


def point_rotate_z(pt, angle):
    # 绕z轴逆时针旋转
    # print(pt, angle)
    rad = angle/180.0*math.pi
    x = pt[0]*math.cos(rad) - pt[1]*math.sin(rad)
    y = pt[0]*math.sin(rad) + pt[1]*math.cos(rad)
    z = pt[2]
    return [x, y, z]


class Solver:
    def __init__(self, config):
        self.kLegJ1ToJ2 = 0.045
        self.kLegJ2ToJ3 = 0.075
        self.kLegJ3ToTip = 0.145
        self.config = config
        self.LEG_ORIGINS = config.LEG_ORIGINS
        self.default_stance = config.default_stance
        add_z = np.array([0, 0, config.default_z_ref])
        add_z = add_z[:, np.newaxis]
        self.tip_to_leg = config.default_stance - \
            config.LEG_ORIGINS
        print(self.LEG_ORIGINS)
        print(self.default_stance)
        print(self.tip_to_leg)

    def ik(self, to):
        angles = []
        # print(to)
        # x, y, _ = to
        # 坐标轴转换，传入坐标横向为y轴，纵向为x轴，与计算相反
        y, x, _ = to
        # 首先要进行坐标变换改变参考系得到相对坐标
        # 以mountPoint即J1为坐标原点，初始角度为x轴，投影到xOy平面上计算第一个角
        angles.append(-math.atan2(y, x))

        # 以J2为坐标原点，把j2-j3-tip 这个三角形投影到xOz平面计算第二、三角
        x = math.sqrt(x*x + y*y) - self.kLegJ1ToJ2
        y = to[2]
        # j2看tip的仰角
        ar = math.atan2(y, x)
        lr2 = x*x + y*y
        lr = math.sqrt(lr2)
        # 余弦定理,j2-j3-tip 这个三角形
        # 角j3-j2-tip
        a1 = math.acos((lr2 + self.kLegJ2ToJ3*self.kLegJ2ToJ3 -
                        self.kLegJ3ToTip*self.kLegJ3ToTip)/(2*self.kLegJ2ToJ3*lr))
        # 角j3-tip-j2
        a2 = math.acos((lr2 - self.kLegJ2ToJ3*self.kLegJ2ToJ3 +
                        self.kLegJ3ToTip*self.kLegJ3ToTip)/(2*self.kLegJ3ToTip*lr))

        angles.append(ar + a1)
        angles.append(math.pi/2 - (a1 + a2))
        # print('ar', to_angle(ar), 'a1', to_angle(a1), 'a2', to_angle(a2))
        return angles

    def ki(self, angles):
        # 以mountPoint即J1为坐标原点，初始角度为x轴
        l = self.kLegJ1ToJ2+self.kLegJ2ToJ3 * \
            math.cos(angles[1]/180.0*math.pi) + self.kLegJ3ToTip * \
            math.sin(angles[2]/180.0*math.pi)
        x = math.cos((angles[0])/180.0*math.pi) * l
        y = -math.sin((angles[0])/180.0*math.pi) * l
        z = self.kLegJ2ToJ3 * \
            math.sin(angles[1]/180.0*math.pi) - self.kLegJ3ToTip * \
            math.cos(angles[2]/180.0*math.pi)
        return [x, y, z]

    def leg_ik(self, leg_index, to):
        # print(to)
        # to_ = self.tip_to_leg[:, leg_index]+to
        rotate_angle = defaultAngle[leg_index]
        return self.ik(point_rotate_z(to, rotate_angle))

    def leg_ki(self, to):
        pass

    def inverse_kinematics_leg(self, foot_locations, config=None):
        alpha = np.zeros((3, 6))
        for i in range(6):
            alpha[:, i] = self.leg_ik(
                i, foot_locations[:, i]
            )
        return alpha

    def inverse_kinematics_body(self, foot_locations, config=None):
        # print(foot_locations)
        alpha = np.zeros((3, 6))
        for i in range(6):
            alpha[:, i] = self.leg_ik(
                i, foot_locations[:, i] - self.LEG_ORIGINS[:, i]
            )
        return alpha


if __name__ == '__main__':
    from config import Configuration
    config = Configuration()
    solver = Solver(config)
    leg_locations = (
        config.default_stance - config.LEG_ORIGINS
        + np.array([0, 0, config.default_z_ref])[:, np.newaxis]
    )
    for i in range(6):
        rad = solver.leg_ik(i, leg_locations[:, i])
        print(i, np.array(rad)/math.pi*180)
