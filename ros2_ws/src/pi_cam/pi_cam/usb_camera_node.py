#!/usr/bin/python
#!coding:utf-8
# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py
import time
import yaml
import os
import cv2
import numpy as np
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.srv import SetCameraInfo
from pi_interface.srv import SetInt32
from pi_interface.srv import GetStrings, SetString
from pi_interface.srv import GetFrame, GetCompressedFrame

from .usb_camera import UsbCamera


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info("[%s] Initializing......" % (self.get_name()))
        self.rate = 30
        # self.size = (480,360)
        self.bridge = CvBridge()
        self.frame_id = "camera_optical_frame"
        self.camera = UsbCamera(rate=self.rate, callback=self.cbImg)
        self.cv_image = None

        self.cali_file_folder = os.path.expanduser(
            '~')+"/Lepi_Data/ros/camera/calibrations/"
        self.cali_file = "default.yaml"
        self.image_msg = None  # Image()
        self.compressed_msg = None  # CompressedImage()
        self.update_frequence = 15
        # self.pub_camera_info = self.create_publisher(CameraInfo, "camera_info", 1)

        self.pub_compressed = self.create_publisher(CompressedImage,
                                                    '~/image_raw/compressed', 1)
        self.create_service(SetInt32, '~/camera_set_enable',
                            self.srvCameraSetEnable)
        self.create_service(SetInt32, '~/camera_set_flip', self.srvCameraSetFlip)
        self.create_service(SetInt32, '~/camera_set_rectify',
                            self.srvCameraSetRectify)
        self.create_service(GetFrame, '~/camera_get_frame',
                            self.srvCameraGetFrame)
        self.create_service(GetCompressedFrame, '~/camera_get_compressed',
                            self.srvCameraGetCompressedFrame)
        self.create_service(GetStrings, '~/get_image_topics',
                            self.srvGetImageTopics)
        self.create_service(GetStrings, '~/list_cali_file',
                            self.srvListCaliFile)
        self.create_service(SetString, '~/load_cali_file',  self.srvLoadCaliFile)
        self.create_service(SetCameraInfo, '~/set_camera_info',
                            self.srvSetCameraInfo)
        self.create_service(SetInt32, '~/set_update_frequence',
                            self.srvSetUpdateFrequence)

        self.declare_parameter('~/calibrate', False)
        calibrate = self.get_parameter(
            '~/calibrate').get_parameter_value().bool_value
        if calibrate:
            self.pub_image = self.create_publisher(Image,
                                                   "~/image",  1)
            self.camera.open_camera(0)
        else:
            self.pub_image = None

        self.timer = self.create_timer(
            1.0/self.update_frequence, self.cbPublish)
        self.get_logger().info("[%s] Initialized......" % (self.get_name()))

    def srvCameraSetEnable(self, request, response):
        response.port = request.value
        if request.value == 1 and self.camera.active == False:
            ret = self.camera.open_camera(request.port)
            response.value = ret
        elif request.value == 0:
            self.camera.active = False
        return response

    def srvCameraSetFlip(self, request, response):
        self.camera.setFlip(request.value)
        response.value = request.value
        return response

    def srvCameraSetRectify(self, request, response):
        response.value = request.value
        if request.value == 1:
            self.camera.setRectify(True)
        elif request.value == 0:
            self.camera.setRectify(False)
        return response

    def srvCameraGetFrame(self, request, response):
        if self.image_msg is None:
            return response
        response.image = self.image_msg
        return response

    def srvCameraGetCompressedFrame(self, request, response):
        if self.compressed_msg is None:
            return response
        response.image = self.compressed_msg
        return response

    def cbImg(self, cv_image):
        self.cv_image = cv_image

    def cbPublish(self, channel=None):
        if self.camera.active == False or self.cv_image is None:
            return
        cv_image = self.camera.getImage()
        if self.pub_image is not None:
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.pub_image.publish(image_msg)

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
        # Publish new image
        self.compressed_msg = msg
        self.pub_compressed.publish(msg)

    def srvGetImageTopics(self, request, response):
        topics = self.get_published_topics()
        for topic in topics:
            print(topic)
            if topic[1] == 'sensor_msgs/CompressedImage':
                response.data.append(topic[0])
        return response

    def srvSetCameraInfo(self, request, response):
        # TODO: save req.camera_info to yaml file
        self.get_logger().info("[cbSrvSetCameraInfo] Callback!")
        # filename = self.cali_file_folder + "pi_cam_%dx%d" % (480,360) + ".yaml"
        filename = self.cali_file_folder + \
            time.strftime('%Y-%m-%d_%H-%M-%S') + ".yaml"
        response.success = self.saveCameraInfo(request.camera_info, filename)
        response.status_message = "Write to %s" % filename  # TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        self.get_logger().info("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
                 'image_height': camera_info_msg.height,
                 'camera_name': self.get_name().strip("/"),  # TODO check this
                 'distortion_model': camera_info_msg.distortion_model,
                 'distortion_coefficients': {'data': camera_info_msg.D, 'rows': 1, 'cols': 5},
                 'camera_matrix': {'data': camera_info_msg.K, 'rows': 3, 'cols': 3},
                 'rectification_matrix': {'data': camera_info_msg.R, 'rows': 3, 'cols': 3},
                 'projection_matrix': {'data': camera_info_msg.P, 'rows': 3, 'cols': 4}}

        self.get_logger().info("[saveCameraInfo] calib %s" % (calib))

        try:
            with open(filename, 'w') as f:
                yaml.safe_dump(calib, f)
                return True
        except IOError:
            return False

    def srvListCaliFile(self, request, response):
        response.data = os.listdir(self.cali_file_folder)
        return response

    def srvLoadCaliFile(self, request, response):
        try:
            self.camera.loadCaliFile(request.data)
            response.data = '加载成功'
        except Exception as e:
            response.data = '加载失败'
        return response

    def srvSetUpdateFrequence(self, request, response):
        print(request)
        if request.value >= 1 and request.value <= 30:
            self.update_frequence = request.value
            self.timer.cancel()
            self.timer = self.create_timer(
                1.0/self.update_frequence, self.cbPublish)
            response.port = 1
            response.value = request.value
        return response

def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
