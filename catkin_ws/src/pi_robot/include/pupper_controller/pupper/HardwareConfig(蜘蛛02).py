
import numpy as np

MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
[[0.0, -5.0, 9.0, -10.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
,dtype = 'float')

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}
