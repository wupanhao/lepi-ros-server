import cv2
import numpy as np
from camera_utils import load_camera_info_2
class Test():
	def __init__(self):
		self.camera_info_msg = load_camera_info_2('../camera_info/calibrations/pi_cam_640x480.yaml')
		# self.camera_info_msg = load_camera_info_2('../camera_info/calibrations/default.yaml')
		K = np.array(self.camera_info_msg.K).reshape((3,3))
		# D = np.array(self.camera_info_msg.D[:4])
		D = np.array([0.,0.,0.,0.])
		P = np.array(self.camera_info_msg.P).reshape((3,4))
		DIM = (self.camera_info_msg.width,self.camera_info_msg.height)
		K = np.array([[303.646249, 0.000000, 326.026328],[ 0.000000, 309.237102, 248.168564],[ 0.000000, 0.000000, 1.000000]])
		# self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
		img = cv2.imread('/home/ubuntu/test_calibration.png')
		# print(img)
		cv2.imshow('raw',img)
		img_undistorted = cv2.fisheye.undistortImage(img, K, D=D, Knew=K)

		cv2.imshow('rect',img_undistorted)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

if __name__ == '__main__':
	t = Test()