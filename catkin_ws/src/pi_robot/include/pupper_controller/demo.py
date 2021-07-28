import numpy as np
import time
from src.Controller import Controller
from src.State import State
from src.Command import Command
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics

import threading
import cv2
from face_recognizer import UltraFaceInference

def main(use_imu=False):
    """Main program
    """

    hardware_interface = HardwareInterface()

    # Create config
    config = Configuration()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    command = Command()
    state.quat_orientation = np.array([1, 0, 0, 0])

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    # exit()

    def get_command():
        return command

    def robot_loop():
        last_loop = time.time()
        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Parse the joystick commands and then update the robot controller's parameters
            command = get_command()

            # Step the controller forward by dt
            controller.run(state, command)

            angles = hardware_interface.set_actuator_postions(
                state.joint_angles)

    loop = threading.Thread(target=robot_loop)
    loop.daemon = True
    loop.start()

    cap = cv2.VideoCapture(0)
    detector = UltraFaceInference()
    while True:
        _, img_ori = cap.read()
        time_time = time.time()
        boxes, labels, probs = detector.detect(img_ori)
        if len(boxes) > 0:
            # print(boxes[0])
            left,top,right,bottom = boxes[0]
            center = [(left+right)/2,(top+bottom)/2]
            if center[0] < 220:
                command.yaw_rate -= 0.05
            elif center[0] > 420:
                command.yaw_rate += -0.05
            else:
                command.yaw_rate = 0
        img_ori = detector.drawBoxes(img_ori, boxes)
        print("inference time: {} s".format(round(time.time() - time_time, 4)))
        cv2.imshow("ultra_face_inference", img_ori)
        c = cv2.waitKey(2)
        if c == 27:
            break
    cv2.destroyAllWindows()
    cap.release()

main()
