from pi_driver import SServo, Servo
import config
from kinematics import tip_to_leg, ik
from movement import path_generator


class HexapodController:
    def __init__(self):
        self.defaultPosition = config.defaultPosition
        self.defaultAngle = config.defaultAngle
        self.center = 1023/2
        self.speed = 1000
        self.attatchServos()
        # self.cbPosition()

    def attatchServos(self, port=None):
        try:
            self.servos = SServo(port)
        except Exception as e:
            self.servos = None
            print(e)
            exit()

    def cbServoAngles(self, angles):
        # for i in [2,5,8,11,14,17]:
        #    positions[i] = - positions[i]
        servo_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9,
                     16, 17, 18, 13, 14, 15, 10, 11, 12]
        for i in range(6):
            angles[i*3+2] = -angles[i*3+2]
        positions = [int(angles[i]/200.0*1023) for i in range(len(angles))]
        servos = [Servo(servo_ids[i], positions[i]+self.center, speed=self.speed)
                  for i in range(len(angles))]
        if self.servos is not None:
            self.servos.set_positions_sync(servos)

    def cbPosition(self, position=[0, 0, 0]):
        path = path_generator()
        print(legs)
        angles = []
        for i in range(6):
            angles.extend(legs[i])
        self.cbServoAngles(angles)

    def test(self):
        seq = path_generator()
        for i in range(len(seq[0])):
            angles = []
            for j in range(6):
                leg = tip_to_leg(j, seq[j][i])
                print(j, leg)
                angle = ik(leg)
                angles.extend(angle)
            self.cbServoAngles(angles)
            time.sleep(0.05)


if __name__ == '__main__':
    import math
    import time
    controller = HexapodController()
    time.sleep(1)
    for i in range(5):
        controller.test()
