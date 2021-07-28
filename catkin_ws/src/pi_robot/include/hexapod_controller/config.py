#!coding:utf-8
import numpy as np
from enum import Enum


class State:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = -0.102555
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0
        self.behavior_state = BehaviorState.REST

        self.quat_orientation = np.array([1, 0, 0, 0])
        self.ticks = 0
        self.foot_locations = np.zeros((3, 6))
        self.joint_angles = np.zeros((3, 6))

        self.behavior_state = BehaviorState.REST


class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    HOP = 2
    FINISHHOP = 3


class Configuration:
    def __init__(self):

        #################### COMMANDS ####################
        factor = 1
        # 速度时间尽量小于 0.4x0.32
        self.max_x_velocity = 0.35/factor
        self.max_y_velocity = 0.35/factor
        self.max_yaw_rate = 1.0
        self.max_roll = 30.0 * np.pi / 180.0
        self.max_pitch = 30.0 * np.pi / 180.0
        self.max_yaw = 30.0 * np.pi / 180.0

        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.09  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = np.pi/6  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 0.6
        self.max_stance_yaw_rate = 1.0

        #################### LEG_ORIGINS ####################
        # 183/2 mm
        self.kLegMiddleX = 0.0915  # m
        # 117/2
        self.kLegOtherX = 0.0585
        # 237/2
        self.kLegOtherY = 0.1185

        self.kLegJ1ToJ2 = 0.045
        self.kLegJ2ToJ3 = 0.075
        self.kLegJ3ToTip = 0.145

        #################### STANCE ####################
        self.delta_x = 0.1
        self.delta_y = 0.09
        self.x_shift = 0.0

        self.middle_x = 0.238976
        self.other_x = 0.16278
        self.other_y = 0.22278
        self.default_z_ref = -0.102555

        #################### SWING ######################
        self.z_coeffs = None
        self.z_clearance = 0.05
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.dt = 0.03
        self.num_phases = 2
        self.contact_phases = np.array(
            [[0, 1],
             [1, 0],
             [0, 1],
             [1, 0],
             [0, 1],
             [1, 0]]
        )

        self.overlap_time = (
            0.3*factor  # duration of the phase where all feet are on the ground
        )
        self.swing_time = (
            0.3*factor  # duration of the phase when only half feet are on the ground
        )

        # 纵向为x轴，横向为y轴
        self.LEG_ORIGINS = np.array(
            [
                [self.kLegOtherY, 0, -self.kLegOtherY,
                 -self.kLegOtherY, 0, self.kLegOtherY],
                [self.kLegOtherX, self.kLegMiddleX, self.kLegOtherX,
                 -self.kLegOtherX, -self.kLegMiddleX, -self.kLegOtherX],
                [0, 0, 0, 0, 0, 0],
            ]
        )

    @ property
    def default_stance(self):
        return np.array(
            [
                [self.other_y, 0, -self.other_y, -self.other_y, 0, self.other_y],
                [
                    self.other_x, self.middle_x, self.other_x,
                    -self.other_x, -self.middle_x, -self.other_x,
                ],
                [0, 0, 0, 0, 0, 0],
                # [self.default_z_ref, self.default_z_ref, self.default_z_ref, self.default_z_ref, self.default_z_ref, self.default_z_ref],
            ]
        )

    ################## SWING ###########################
    @ property
    def z_clearance(self):
        return self.__z_clearance

    @ z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    ########################### GAIT ####################
    @ property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @ property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @ property
    def stance_ticks(self):
        return self.overlap_ticks

    @ property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks]
        )

    @ property
    def phase_length(self):
        return self.overlap_ticks + self.swing_ticks


class Command:
    """Stores movement command
    """

    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = -0.102555  # Config.default_z_ref
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0

        self.hop_event = False
        self.trot_event = False
        self.activate_event = False


if __name__ == '__main__':
    config = Configuration()
    print(config.z_clearance)
    config.z_clearance = 0.07
    print(config.z_clearance)
