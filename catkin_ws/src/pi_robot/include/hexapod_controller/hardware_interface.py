
import math
from sservo import SServo, Servo
ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]


class HardwareInterface:
    def __init__(self, neutral_angle_degrees=None):
        # self.pi = pigpio.pi()
        self.pi = SServo()
        # self.pwm_params = PWMParams()
        self.neutral_angle_degrees = neutral_angle_degrees
        # self.servo_states = [0 for i in range(12)]
        self.servo_ids = [i+1 for i in range(18)]
        self.servo_angles = [0 for i in range(18)]

    def set_actuator_postions(self, joint_angles):
        angles = []
        servos = []
        for leg_index in range(6):
            for axis_index in range(3):
                i = leg_index*3+axis_index
                angle = joint_angles[axis_index][leg_index]/math.pi*180
                self.servo_angles[ids[i]-1] = int(angle)
                pos = angle+self.neutral_angle_degrees[axis_index][leg_index]
                if axis_index == 2:
                    pos = -pos
                angles.append(int(angle))
                servos.append(
                    Servo(ids[i], int(pos/200.0*1023+1023/2), speed=2000))

        # servos = [Servo(ids[i], int(angles[i]/200.0*1023+1023/2), speed=2000)
        #           for i in range(18)]
        self.pi.set_positions_sync(servos)
        return ids, angles

    def set_actuator_angles(self, angles):
        pass
        # angles = []
        ids = []
        for leg_index in range(4):
            for axis_index in range(3):
                id = leg_index*3+axis_index+1
                ids.append(id)
        servos = [Servo(ids[i], int(angles[i]/200.0*1023+1023/2), speed=2000)
                  for i in range(12)]
        self.pi.set_positions_sync(servos)
