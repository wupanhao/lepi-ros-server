import numpy as np
import time
# from src.IMU import IMU
from joystick_interface import JoystickInterface
from hardware_interface import HardwareInterface
from hardware_config import NEUTRAL_ANGLE_DEGREES

from kinematics import Solver
from controller import Controller
from config import Configuration, State


def main(use_imu=False):
    """Main program
    """
    # Create config
    config = Configuration()
    solver = Solver(config)
    hardware_interface = HardwareInterface(NEUTRAL_ANGLE_DEGREES)

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        solver.inverse_kinematics_body,
    )
    state = State()
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    # exit()

    # Wait until the activate button has been pressed
    while True:
        print("Waiting for L1 to activate robot.")
        while True:
            # break
            command = joystick_interface.get_command(state, True)
            # print(command)
            # joystick_interface.set_color(config.ps4_deactivated_color)
            if command.activate_event == 1:
                break
            time.sleep(0.1)
        print("Robot activated.")
        # joystick_interface.set_color(config.ps4_color)

        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Deactivating Robot")
                break

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            try:
                controller.run(state, command)
            except Exception as e:
                print(e)

            print(hardware_interface.set_actuator_postions(state.joint_angles))
            # time.sleep(2)


main()
