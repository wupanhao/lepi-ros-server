#!/usr/bin/python

import os.path
import numpy as np
import cv2
import time
import rospkg

from camera_utils import load_camera_info_2

class ImageRector:
    def __init__(self):
        # Get path to calibration yaml file
        self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"

        # Load calibration yaml file
        self.camera_info_msg = load_camera_info_2(self.cali_file)

        K = np.array(self.camera_info_msg.K).reshape((3,3))
        # D = np.array(self.camera_info_msg.D[:4])
        D = np.array([0.,0.,0.,0.])
        # P = np.array(self.camera_info_msg.P).reshape((3,4))
        DIM = (self.camera_info_msg.width,self.camera_info_msg.height)
        self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

    def rect(self, cv_image):
        rect_image = cv2.remap(cv_image,self.mapx, self.mapy,cv2.INTER_LINEAR)
        return rect_image

if __name__ == '__main__':
    rector = ImageRector()
    test_image = cv2.imread('./test_norect.png')
    rect_image = rector.rect(test_image)
    cv2.imwrite('./test_rect.png',rect_image)