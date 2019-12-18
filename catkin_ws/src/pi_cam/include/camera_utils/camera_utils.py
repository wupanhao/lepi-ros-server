import yaml
import cv2
import numpy as np

# from cam_info_reader_node
from sensor_msgs.msg import CameraInfo

def bgr_from_jpg(data):
    """ Returns an OpenCV BGR image from a string """
    s = np.fromstring(data, np.uint8)
    bgr = cv2.imdecode(s, cv2.IMREAD_COLOR)
    if bgr is None:
        msg = 'Could not decode image (cv2.imdecode returned None). '
        msg += 'This is usual a sign of data corruption.'
        raise ValueError(msg)
    return bgr

def load_camera_info_2(filename):
    stream = file(filename, 'r')
    calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    return cam_info