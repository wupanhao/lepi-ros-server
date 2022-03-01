#!coding:utf-8
from scipy.spatial.transform import Rotation as R
# from apriltags3 import Detector
from dt_apriltags import Detector
from camera_utils import load_camera_info_3, cameraList
import cv2
import numpy as np
import os


class ApriltagDetector:
    """
    ApriltagDetector类, 用来检测特定的Apriltag标签
    Attributes:
    cali_file: str 广角摄像头矫正文件路径，如果想做姿态估计需要用到
    """

    def __init__(self):
        cur_dir = os.path.dirname(os.path.abspath(__file__))
        # self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/default.yaml"
        # self.cali_file = "default.yaml"
        self.camera_info_msg = load_camera_info_3()
        # self.detector = Detector(
        # print(cur_dir + '/../../../../devel_isolated/apriltag/lib')
        # self.detector = Detector(searchpath=[cur_dir + '/../../../../devel_isolated/apriltag/lib'],
        self.detector = Detector(
            # self.detector = Detector(families='tag36h11',
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0)
        self.cameraMatrix = np.array(
            self.camera_info_msg.K).reshape((3, 3))
        self.camera_params = (
            self.cameraMatrix[0, 0], self.cameraMatrix[1, 1], self.cameraMatrix[0, 2], self.cameraMatrix[1, 2])
        self.image = None
        self.detections = []
        self.tags = []
        self.tag = None

        if hasattr(R, 'from_dcm'):
            self.from_dcm_or_matrix = R.from_dcm
        else:
            self.from_dcm_or_matrix = R.from_matrix

    def detect(self, cv_image, label_tags=True, tag_size=0.065):
        """
        检测函数
        Keyword arguments:
        cv_image: image 原图
        tag_size：float Apriltag标签的尺寸，以米为单位,估算距离需要
        Returns:
        tags: [Detection object] 检测到的Apriltag
                Detection object:
                tag_family = tag36h11
                tag_id = 5
                hamming = 0
                decision_margin = 75.3475112915
                homography = [[-4.04986020e+00 -8.25442426e+00  7.03246452e+02]
                [ 1.60941194e+01  2.64459780e+00  2.71387964e+02]
                [-3.94154088e-03  1.31415634e-02  1.00000000e+00]]
                center = [703.24645207 271.38796427]
                corners = [[687.30065918 253.60606384]
                [684.64343262 287.48184204]
                [719.746521   289.78796387]
                [722.19494629 254.99520874]]
                pose_R = [[-0.05079941 -0.99821651  0.03135634]
                [ 0.99851052 -0.05139001 -0.01832521]
                [ 0.01990393  0.03037872  0.99934027]]
                pose_t = [[0.72577546]
                [0.04402775]
                [0.58391835]]
                pose_err = 5.94095039944e-07
        """
        if len(cv_image.shape) == 3:
            image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            image_gray = cv_image
        self.tags = self.detector.detect(
            image_gray, True, self.camera_params, tag_size)  # tag size in meter
        # if label_tags:
        #     self.label_tags(cv_image, self.tags)
        self.image = cv_image
        self.detections = self.toApriltagDetections()
        if len(self.detections) > 0:
            self.tag = self.detections[0]
        else:
            self.tag = None
        return cv_image

    def label_tags(self, rect_image, tags=None):
        if tags is None:
            tags = self.tags
        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(rect_image, tuple(
                    tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
            cv2.putText(rect_image, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int)+10,
                             tag.corners[0, 1].astype(int)+10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8, color=(0, 0, 255))

    def toApriltagDetections(self, tags=None):
        if tags is None:
            tags = self.tags
        detections = []
        for tag in tags:
            r = self.from_dcm_or_matrix(np.array(tag.pose_R))
            offset = np.array(tag.pose_t)*100
            euler = r.as_euler('xyz', degrees=True)
            detection = (tag.tag_id, euler, offset)
            detections.append(detection)
        return detections

    def isDetected(self, id=None):
        if id is None and len(self.detections) > 0:
            return True
        else:
            for tag in self.detections:
                if id == tag[0]:
                    self.tag = tag
                    return True
        self.tag = None
        return False


if __name__ == '__main__':
    import time
    detector = ApriltagDetector()
    # import os
    # os.system(
    #     'wget https://april.eecs.umich.edu/media/apriltag/apriltagrobots_overlay.jpg -O /tmp/test.jpg')
    # test_image = cv2.imread('/tmp/test.jpg')
    # print(detector.detect(test_image))
    dev = cameraList()
    cap = cv2.VideoCapture(dev[0])
    while True:
        # Grab a single frame of video
        ret, frame = cap.read()
        start = time.time()
        detector.detect(frame)
        end = time.time()
        # detector.label_tags(frame, tags)
        print("detect %d frame in %.2f ms" %
              (1, (end - start)*1000))
        cv2.imshow("images", frame)
        c = cv2.waitKey(4)
        if c == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
