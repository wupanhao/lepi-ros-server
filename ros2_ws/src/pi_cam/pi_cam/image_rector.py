#!/usr/bin/python

import numpy as np
import cv2
import time
# import rospkg

from .camera_utils import load_camera_info_3

class ImageRector:
    def __init__(self,size=(640,480),cali_file="default.yaml"):
        # Get path to calibration yaml file
        # self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
        # self.cali_file = "default.yaml"

        # Load calibration yaml file
        self.camera_info_msg = load_camera_info_3(cali_file)

        K = np.array(self.camera_info_msg.K).reshape((3,3))
        D = np.array(self.camera_info_msg.D[:4])
        R = np.array(self.camera_info_msg.R).reshape((3,3))
        # D = np.array([0.,0.,0.,0.])
        # P = np.array(self.camera_info_msg.P).reshape((3,4))
        # DIM = (self.camera_info_msg.width,self.camera_info_msg.height)
        print('ImageRector',size,cali_file)
        ncm, _ = cv2.getOptimalNewCameraMatrix(K, D, size,0.0)
        # self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), ncm, size, cv2.CV_16SC2)
        # self.mapx, self.mapy = cv2.initUndistortRectifyMap(K, D, R, ncm, size, cv2.CV_32FC1)
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(K, D, R, ncm, size, cv2.CV_16SC2)

    def rect(self, cv_image):
        rect_image = cv2.remap(cv_image,self.mapx, self.mapy,cv2.INTER_LINEAR)
        return rect_image

if __name__ == '__main__':
    rector = ImageRector()
    #test_image = cv2.imread('./test_norect.png')
    #rect_image = rector.rect(test_image)
    #cv2.imwrite('./test_rect.png',rect_image)
    cap = cv2.VideoCapture(0)
    while True:
        # Grab a single frame of video
        ret, frame = cap.read()
        start = time.time()
        rect_image = rector.rect(frame)
        end = time.time()
        print("rect 1 frame in %.2f ms" %
              ( (end - start)*1000))
        cv2.imshow("images", rect_image)
        c = cv2.waitKey(4)
        if c == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
