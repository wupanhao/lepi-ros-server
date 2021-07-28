
import time
from flask import Flask, jsonify, request, json
from flask_cors import CORS
import numpy as np
import math
from hardware_interface import HardwareInterface
from joystick_interface import JoystickInterface
from controller import Controller
from config import Configuration, State
from kinematics import Solver
# from src.IMU import IMU
try:
    from hardware_config import NEUTRAL_ANGLE_DEGREES
except Exception as e:
    print(e)
    NEUTRAL_ANGLE_DEGREES = np.zeros((3, 6))
angles = []
for i in range(6):
    for j in range(3):
        angles.append(NEUTRAL_ANGLE_DEGREES[j, i])

# Create config
config = Configuration()
solver = Solver(config)
hardware_interface = HardwareInterface(NEUTRAL_ANGLE_DEGREES)


def toConfig(angles):
    res = np.zeros((3, 6))
    for i in range(6):
        for j in range(3):
            res[j][i] = angles[i*3+j]
    return res.tolist()


def toArray(res):
    angles = [0 for i in range(18)]
    for i in range(3):
        for j in range(6):
            angles[j*3+i] = res[i][j]
    return angles


print(angles)
template = '''
import numpy as np

MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
SERVO_ARRAY_PLACEHOLDER
,dtype = 'float')

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}
'''


app = Flask(__name__)
CORS(app, supports_credentials=True)


@app.route('/')
def index():
    with open('./index.html', 'r') as f:
        return f.read()


@app.route('/get')
def get():
    return jsonify(angles)


@app.route('/set')
def set():
    global angles
    data = request.args.get('angles')
    try:
        array = json.loads(data)
        if type(array) == list and len(array) == 18:
            angles = array
    except Exception as e:
        print(e)

    zeros = np.zeros((3, 6))
    hardware_interface.neutral_angle_degrees = np.array(
        toConfig(angles))
    hardware_interface.set_actuator_postions(zeros)
    return jsonify(angles)


@app.route('/save')
def save():
    try:
        with open('./hardware_config.py', 'w') as f:
            f.write(template.replace(
                'SERVO_ARRAY_PLACEHOLDER', str(toConfig(angles))))
    except Exception as e:
        print(e)
        return jsonify({"status": 1})
    return jsonify({"status": 0})


def main(use_imu=False):
    """Main program
    """

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
                time.sleep(config.dt-(now - last_loop))
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
            controller.run(state, command)

            # Update the pwm widths going to the servos
            angles = []
            for leg in state.joint_angles:
                for i in leg:
                    angles.append(int(i/math.pi*180))
            # print(angles)
            # print(angles, state.joint_angles)
            print(hardware_interface.set_actuator_postions(state.joint_angles))
            # time.sleep(2)


if __name__ == '__main__':
    import threading
    threading._start_new_thread(main, ())
    app.run(port=8080, host='0.0.0.0')
