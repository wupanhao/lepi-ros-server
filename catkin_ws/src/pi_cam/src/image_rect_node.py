import cv2
import numpy as np
import time
import rospkg
import rospy
# import matplotlib.pylab as plt

from line_detector.timekeeper import TimeKeeper
from sensor_msgs.msg import Image

from camera_utils import load_camera_info_2

class Test():
    def __init__(self):
        self.node_name = rospy.get_name()
        self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
        self.camera_info_msg = load_camera_info_2(self.cali_file)
        rospy.loginfo("[%s] CameraInfo: %s" % (self.node_name, self.camera_info_msg))
        K = np.array(self.camera_info_msg.K).reshape((3,3))
        D = np.array(self.camera_info_msg.D)
        D = np.array([0.,0.,0.,0.])
        # P = np.array(self.camera_info_msg.P).reshape((3,4))
        DIM = (self.camera_info_msg.width,self.camera_info_msg.height)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        

if __name__ == '__main__':
	rospy.init_node('image_rect_node',anonymous=False)
	test = Test()



