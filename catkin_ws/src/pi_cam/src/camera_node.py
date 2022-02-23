#!/usr/bin/python
#!coding:utf-8
import time
# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py
from cv_bridge import CvBridge
import rospy
import yaml
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

# from camera_utils import load_camera_info_3

from pi_driver.srv import SetInt32, SetInt32Response
from pi_driver.srv import GetStrings, GetStringsResponse, SetString, SetStringResponse
from pi_cam.srv import GetFrame, GetFrameResponse, GetCompressedFrame, GetCompressedFrameResponse
# from std_msgs.msg import Empty

from camera_utils import UsbCamera
import os
import cv2
import numpy as np
import time


class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.is_shutdown = False
        self.rectify = False
        self.rate = 30
        # self.size = (480,360)
        self.bridge = CvBridge()
        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"
        self.camera = UsbCamera(rate=self.rate, callback=self.cbImg)
        self.cv_image = None
        self.lost_count = 0

        # self.r = rospy.Rate(self.rate)
        # self.rector = ImageRector(self.size)
        # self.rector = ImageRector((640,480))
        # self.cali_file_folder = os.path.dirname(os.path.abspath(__file__)) + "/../camera_info/calibrations/"
        self.cali_file_folder = os.path.expanduser(
            '~')+"/Lepi_Data/ros/camera/calibrations/"
        self.cali_file = "default.yaml"
        # self.camera_info_msg = load_camera_info_3()
        # self.camera_info_msg_rect = load_camera_info_3()
        self.image_msg = None  # Image()
        self.compressed_msg = None  # CompressedImage()
        self.last_publish = 0
        self.update_frequence = 15
        self.last_update = time.time()
        # self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)

        # self.pub_rect = rospy.Publisher("~image_rect", Image, queue_size=1)
        self.pub_camera_info = rospy.Publisher(
            "~camera_info", CameraInfo, queue_size=1)

        self.pub_compressed = rospy.Publisher(
            "~image_raw/compressed", CompressedImage, queue_size=1)
        # self.pub_camera_info_rect = rospy.Publisher("~rect/camera_info", CameraInfo, queue_size=1)
        rospy.Service('~camera_set_enable', SetInt32, self.srvCameraSetEnable)
        rospy.Service('~camera_set_flip', SetInt32, self.srvCameraSetFlip)
        rospy.Service('~camera_set_rectify', SetInt32,
                      self.srvCameraSetRectify)
        rospy.Service('~camera_get_frame', GetFrame, self.srvCameraGetFrame)
        rospy.Service('~camera_get_compressed', GetCompressedFrame,
                      self.srvCameraGetCompressedFrame)
        rospy.Service('~get_image_topics', GetStrings, self.srvGetImageTopics)
        rospy.Service('~list_cali_file', GetStrings, self.srvListCaliFile)
        rospy.Service('~load_cali_file', SetString, self.srvLoadCaliFile)
        rospy.Service("~set_camera_info", SetCameraInfo, self.srvSetCameraInfo)
        rospy.Service('~set_update_frequence', SetInt32,
                      self.srvSetUpdateFrequence)
        # rospy.Subscriber('~shutdown', Empty, self.cbShutdown)
        # self.pid_set_enable_srv = rospy.Service('~pid_set_enable', SetPid, self.cbPidSetEnable)
        # self.pub_image_detection = rospy.Publisher("~image_detections", Image, queue_size=1)

        calibrate = rospy.get_param('~calibrate', False)
        if calibrate:
            self.pub_image = rospy.Publisher("~image", Image, queue_size=1)
            self.camera.open_camera(0)
        else:
            self.pub_image = None

        # reader2 = threading.Thread(target=self.start_update_loop)
        # reader.daemon = True
        # reader2.start()

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.update_frequence), self.cbPublish)
        rospy.loginfo("[%s] Initialized......" % (self.node_name))

    def srvCameraSetEnable(self, params):
        if params.value == 1 and self.camera.active == False:
            ret = self.camera.open_camera(params.port)
            return SetInt32Response(1, ret)
        elif params.value == 0:
            self.camera.active = False
            return SetInt32Response(1, 0)
        return SetInt32Response()

    def srvCameraSetFlip(self, params):
        self.camera.setFlip(params.value)
        return SetInt32Response(0, params.value)

    def srvCameraSetRectify(self, params):
        if params.value == 1:
            self.camera.setRectify(True)
        elif params.value == 0:
            self.camera.setRectify(False)
        return SetInt32Response(1, params.value)

    def srvCameraGetFrame(self, params):
        if self.image_msg is None:
            return GetFrameResponse()
        return GetFrameResponse(self.image_msg)

    def srvCameraGetCompressedFrame(self, params):
        if self.compressed_msg is None:
            return GetCompressedFrameResponse()
        return GetCompressedFrameResponse(self.compressed_msg)

    def cbImg(self, cv_image):
        self.cv_image = cv_image
        # self.last_update = time.time()
        # image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # self.pub_image.publish(image_msg)
        # self.cbPublish()

    def cbPublish(self, channel=None):
        if self.camera.active == False or self.cv_image is None:
            return
        # if self.last_update <= self.last_publish:
        #     return
        # self.last_publish = self.last_update
        cv_image = self.camera.getImage()
        if self.pub_image is not None:
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.pub_image.publish(image_msg)
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_msg.header.stamp = rospy.Time.now()
        image_msg.header.frame_id = self.frame_id
        self.image_msg = image_msg
        # self.pub_raw.publish(image_msg)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
        # Publish new image
        self.compressed_msg = msg
        self.pub_compressed.publish(msg)

    def srvGetImageTopics(self, params):
        topics = rospy.get_published_topics()
        res = GetStringsResponse()
        for topic in topics:
            if topic[1] == 'sensor_msgs/CompressedImage':
                res.data.append(topic[0])
        return res

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." % (self.node_name))
        self.is_shutdown = True
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

    def srvSetCameraInfo(self, req):
        # TODO: save req.camera_info to yaml file
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        # filename = self.cali_file_folder + "pi_cam_%dx%d" % (480,360) + ".yaml"
        filename = self.cali_file_folder + \
            time.strftime('%Y-%m-%d_%H-%M-%S') + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info, filename)
        response.status_message = "Write to %s" % filename  # TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
                 'image_height': camera_info_msg.height,
                 'camera_name': rospy.get_name().strip("/"),  # TODO check this
                 'distortion_model': camera_info_msg.distortion_model,
                 'distortion_coefficients': {'data': camera_info_msg.D, 'rows': 1, 'cols': 5},
                 'camera_matrix': {'data': camera_info_msg.K, 'rows': 3, 'cols': 3},
                 'rectification_matrix': {'data': camera_info_msg.R, 'rows': 3, 'cols': 3},
                 'projection_matrix': {'data': camera_info_msg.P, 'rows': 3, 'cols': 4}}

        rospy.loginfo("[saveCameraInfo] calib %s" % (calib))

        try:
            with open(filename, 'w') as f:
                yaml.safe_dump(calib, f)
                return True
        except IOError:
            return False

    def srvListCaliFile(self, params):
        return GetStringsResponse(os.listdir(self.cali_file_folder))

    def srvLoadCaliFile(self, params):
        try:
            # rector = ImageRector(size=(640,480),cali_file=params.data)
            # self.camera.rector = rector
            self.camera.loadCaliFile(params.data)
            return SetStringResponse('加载成功')
        except Exception as e:
            return SetStringResponse('加载失败')

    def srvSetUpdateFrequence(self, params):
        print(params)
        if params.value >= 1 and params.value <= 30:
            self.update_frequence = params.value
            self.timer.shutdown()
            self.timer = rospy.Timer(rospy.Duration.from_sec(1.0/self.update_frequence), self.cbPublish)
            return SetInt32Response(1, params.value)
        else:
            return SetInt32Response()

    # not accurate
    def start_update_loop(self):
        # print(self.d51_driver.value_updated)
        while not self.is_shutdown:
            self.cbPublish()
            if self.update_frequence == 0:
                time.sleep(1)
            else:
                time.sleep(1.0/self.update_frequence)


if __name__ == '__main__':
    rospy.init_node('camera_node', anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    # thread.start_new_thread(camera_node.startCaptureCompressed, ())
    rospy.spin()
