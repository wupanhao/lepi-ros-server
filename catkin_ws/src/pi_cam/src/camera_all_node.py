#!/usr/bin/env python
import io
import thread
import os
import yaml
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from picamera import PiCamera
from picamera.array import PiRGBArray
import rospkg
import rospy
from sensor_msgs.msg import CompressedImage,Image,CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

from camera_utils import load_camera_info_2

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.framerate_high = self.setupParam("~framerate_high", 30.0)
        self.framerate_low = self.setupParam("~framerate_low", 15.0)
        self.res_w = self.setupParam("~res_w", 640)
        self.res_h = self.setupParam("~res_h", 480)

        self.image_msg = CompressedImage()
        self.bridge = CvBridge()

        # Setup PiCamera
        self.camera = PiCamera()
        # self.framerate = self.framerate_low  # default to low
        # self.framerate = self.framerate_high  # default to high
        self.framerate = 3
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w, self.res_h)

        self.count = 0
        self.decode_frame = 7

        # For intrinsic calibration
        self.cali_file_folder = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/"

        # For load camera info
        self.cali_file_name = self.setupParam("~cali_file_name", "default")
        self.cali_file = rospkg.RosPack().get_path('pi_cam') + "/camera_info/calibrations/" + self.cali_file_name + ".yaml"

        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"

        self.has_published = False
        self.pub_img = rospy.Publisher("~image/compressed", CompressedImage, queue_size=1)
        self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1)
        self.pub_camera_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)
        # self.sub_switch_high = rospy.Subscriber("~framerate_high_switch", BoolStamped, self.cbSwitchHigh, queue_size=1)

        self.camera_info_msg = None
        # Load calibration yaml file
        rospy.logwarn("[%s] Can't find calibration file: %s.\nUsing default calibration instead."
                          % (self.node_name, self.cali_file))

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")
        # Print out and prepare message
        rospy.loginfo("[%s] Using calibration file: %s" % (self.node_name, self.cali_file))
        self.camera_info_msg = load_camera_info_2(self.cali_file)
        self.camera_info_msg.header.frame_id = self.frame_id
        # self.camera_info_msg.header.frame_id = rospy.get_namespace() + "camera_optical_frame"
        rospy.loginfo("[%s] CameraInfo: %s" % (self.node_name, self.camera_info_msg))

        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info", SetCameraInfo, self.cbSrvSetCameraInfo)

        self.stream = io.BytesIO()
        self.is_shutdown = False
        self.update_framerate = False
        # Setup timer
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbSwitchHigh(self, switch_msg):
        print switch_msg
        if switch_msg.data and self.framerate != self.framerate_high:
            self.framerate = self.framerate_high
            self.update_framerate = True
        elif not switch_msg.data and self.framerate != self.framerate_low:
            self.framerate = self.framerate_low
            self.update_framerate = True

    def startCapturingCompresed(self):
        rospy.loginfo("[%s] Start capturing." % (self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            gen = self.grabAndPublish(self.stream, self.pub_img)
            try:
                self.camera.capture_sequence(gen, 'jpeg', use_video_port=True, splitter_port=0)
            except StopIteration:
                pass
            # print "updating framerate"
            self.camera.framerate = self.framerate
            self.update_framerate = False
        self.camera.close()
        rospy.loginfo("[%s] Capture Ended." % (self.node_name))

    def grabAndPublish(self, stream, publisher):
        while not self.update_framerate and not self.is_shutdown and not rospy.is_shutdown():
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()
            # Generate compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = stream_data

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            self.pub_img.publish(image_msg)
            self.count = self.count + 1
            if self.count % self.decode_frame == 0:
                self.decodeAndPublishRaw(image_msg)
                self.publishCameraInfo(image_msg)
            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." % (self.node_name))
                self.has_published = True
            rospy.sleep(rospy.Duration.from_sec(0.001))
    def startCapturingRaw(self):
        rospy.loginfo("[%s] Start capturing." % (self.node_name))
        while not self.is_shutdown and not rospy.is_shutdown():
            gen = self.grabRawAndPublish(self.stream)
            try:
                self.camera.capture_sequence(gen, 'rgb', use_video_port=True, splitter_port=0)
            except StopIteration:
                pass
            # print "updating framerate"
            self.camera.framerate = self.framerate
            self.update_framerate = False
        self.camera.close()
        rospy.loginfo("[%s] Capture Ended." % (self.node_name))
    def grabRawAndPublish(self, stream):
        while not self.update_framerate and not self.is_shutdown and not rospy.is_shutdown():
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()
            # Generate compressed image
            image_msg = Image()
            image_msg.encoding = "rgb8"
            image_msg.data = stream_data
            image_msg.width = 640
            image_msg.height = 480
            image_msg.step = 1920 # 640 x 3
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            self.pub_raw.publish(image_msg)
            self.publishCameraInfo(image_msg)
            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." % (self.node_name))
                self.has_published = True
            rospy.sleep(rospy.Duration.from_sec(0.001))

    def decodeAndPublishRaw(self,image_msg):
        # Publish raw image
        np_arr = np.fromstring(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # time_1 = time.time()
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # time_2 = time.time()
        img_msg.header.stamp = image_msg.header.stamp
        img_msg.header.frame_id = image_msg.header.frame_id
        self.pub_raw.publish(img_msg)        

    def publishCameraInfo(self,image_msg):
        self.camera_info_msg.header.stamp = image_msg.header.stamp
        self.pub_camera_info.publish(self.camera_info_msg)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." % (self.node_name))
        self.is_shutdown = True
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

    def cbSrvSetCameraInfo(self, req):
        # TODO: save req.camera_info to yaml file
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + "pi_cam_%dx%d" % (self.res_w,self.res_h) + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info, filename)
        response.status_message = "Write to %s" % filename  #TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
        'image_height': camera_info_msg.height,
        'camera_name': rospy.get_name().strip("/"),  #TODO check this
        'distortion_model': camera_info_msg.distortion_model,
        'distortion_coefficients': {'data': camera_info_msg.D, 'rows':1, 'cols':5},
        'camera_matrix': {'data': camera_info_msg.K, 'rows':3, 'cols':3},
        'rectification_matrix': {'data': camera_info_msg.R, 'rows':3, 'cols':3},
        'projection_matrix': {'data': camera_info_msg.P, 'rows':3, 'cols':4}}

        rospy.loginfo("[saveCameraInfo] calib %s" % (calib))

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False

if __name__ == '__main__':
    rospy.init_node('pi_cam', anonymous=False)
    camera_node = CameraNode()
    rospy.on_shutdown(camera_node.onShutdown)
    thread.start_new_thread(camera_node.startCapturingCompresed, ())
    rospy.spin()
