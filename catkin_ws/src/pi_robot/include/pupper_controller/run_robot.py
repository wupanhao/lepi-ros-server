#!coding:utf-8
import numpy as np
import time
from src.IMU import IMU
from src.Controller import Controller
from JoystickInterface import JoystickInterface
from src.State import State
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
from dances import bainian

import signal

# 自定义信号处理函数
def my_handler(signum, frame):
    print("进程终止")
    exit()

# 设置相应信号处理的handler
signal.signal(signal.SIGINT, my_handler)
signal.signal(signal.SIGTERM, my_handler)

# Create config
config = Configuration()


def toConfig(angles):
    states = np.zeros((3, 4))
    for i in range(4):
        for j in range(3):
            states[j][i] = angles[i*3+j]
    return states

def main(use_imu=False):
    """Main program
    """

    hardware_interface = HardwareInterface()

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
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
            joystick_interface.set_color(config.ps4_deactivated_color)
            if command.activate_event == 1:
                break
            time.sleep(0.1)
        print("Robot activated.")
        joystick_interface.set_color(config.ps4_color)

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

            if command.bainian_event == 1:
                command.bainian_event = 0
                for angles in bainian:
                    joint_angles = toConfig(angles)/180.0*np.pi
                    hardware_interface.set_actuator_postions(
                        joint_angles)
                    time.sleep(0.5)
            
            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            controller.run(state, command)

            # Update the pwm widths going to the servos
            angles = []
            for leg in state.joint_angles:
                for i in leg:
                    angles.append(int(i/np.pi*180))
            # print(angles)
            # print(angles, state.joint_angles)
            print(hardware_interface.set_actuator_postions(state.joint_angles))
            pub_joint_states(state.joint_angles)
            # time.sleep(2)


def pub_joint_states(joint_angles):
    angles = []
    ids = []
    for leg_index in range(4):
        for axis_index in range(3):
            angles.append(joint_angles[axis_index][leg_index])
    joint_msg.position = angles
    joint_states.publish(joint_msg)


if __name__ == '__main__':
    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node('joystick_controller_node')
    joint_states = rospy.Publisher('/joint_states', JointState)
    joint_msg = JointState()
    joint_msg.name = ["shoulder_joint_rf", "elbow_joint_rf", "wrist_joint_rf",
                      "shoulder_joint_lf", "elbow_joint_lf", "wrist_joint_lf",
                      "shoulder_joint_rb", "elbow_joint_rb", "wrist_joint_rb",
                      "shoulder_joint_lb", "elbow_joint_lb", "wrist_joint_lb",
                      ]
    main()

