from scipy.spatial.transform import Rotation as R
import rospkg
from apriltags3 import Detector
from camera_utils import load_camera_info_2
import cv2
import numpy as np
class ApriltagDetector:
	def __init__(self):
		self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
		self.camera_info_msg = load_camera_info_2(self.cali_file)
		self.detector = Detector(searchpath=['/home/pi/workspace/apriltags3-py/apriltags/lib'],
			families='tag36h11',
			nthreads=1,
			quad_decimate=3.0,
			quad_sigma=0.0,
			refine_edges=1,
			decode_sharpening=0.25,
			debug=0)
		self.cameraMatrix = np.array(self.camera_info_msg.K).reshape((3,3))
		self.camera_params = ( self.cameraMatrix[0,0], self.cameraMatrix[1,1], self.cameraMatrix[0,2], self.cameraMatrix[1,2] )
	def detect(self,cv_image,tag_size=0.065):
		image_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
		tags = self.detector.detect(image_gray, True, self.camera_params, tag_size) # tag size in meter
		return tags

if __name__ == '__main__':
	detector = ApriltagDetector()
	test_image = cv2.imread('./test.png')
	print(detector.detect(test_image))