#!/usr/bin/python3
#!coding:utf-8

import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import FaceDetection
from pi_cam.srv import GetFaceDetections,GetFaceDetectionsResponse
from pi_driver.srv import GetStrings,GetStringsResponse,SetString,SetStringResponse
from pi_driver.srv import SetInt32,SetInt32Response
from camera_utils import bgr_from_jpg,jpg_from_bgr
from face_recognizer import UltraFaceInference
from pi_cam.srv import GetFrame,GetFrameRequest

class UltraFaceInferenceNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.bridge = CvBridge()
        self.visualization = True

        self.image_msg = None
        # self.pub_detections = rospy.Publisher("~image_ultra_face", Image, queue_size=1)
        self.pub_detections = rospy.Publisher("~image_ultra_face", CompressedImage, queue_size=1)
        rospy.Service('~set_threshold', SetInt32, self.cbSetThreshold)
        rospy.Service('~set_resize', SetInt32, self.srvSetResize)
        self.detector = UltraFaceInference()

        rospy.Service('~detect_face_locations', GetFaceDetections, self.cbDetectFaceLocations)
        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg , queue_size=1)
        self.sub_image = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.cbImg ,  queue_size=1)

        # rospy.loginfo("[%s] wait_for_service : camera_get_frame..." % (self.node_name))
        # rospy.wait_for_service('~camera_get_frame')
        # self.get_frame = rospy.ServiceProxy('~camera_get_frame', GetFrame)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbImg(self,image_msg):
        self.image_msg = image_msg

    def cbDetectFaceLocations(self,params):
        # print(params)
        start = time.time()
        # res = self.get_frame(GetFrameRequest())
        # print("get_frame time: {} s".format(round((time.time() - start)*1000, 4)))
        # image_msg = res.image

        image_msg = self.image_msg    
        if image_msg == None:
            return GetFaceDetectionsResponse()
        if hasattr(image_msg,'format'): # CompressedImage
            try:
                cv_image = bgr_from_jpg(image_msg.data)
            except ValueError as e:
                rospy.loginfo('apriltag_detector cannot decode image: %s' % e)
                return GetFaceDetectionsResponse()
        else: # Image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg)
        rect_image = cv_image
        print("decode time: {} s".format(round((time.time() - start)*1000, 4)))
        boxes, labels, probs = self.detector.detect(rect_image)
        print("detect time: {} s".format(round((time.time() - start)*1000, 4)))
        print(boxes,labels,probs)
        if self.visualization:
            faces = self.detector.drawBoxes(rect_image,boxes)
            print("drawBoxes time: {} s".format(round((time.time() - start)*1000, 4)))
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = jpg_from_bgr(faces)
            print("encode time: {} s".format(round((time.time() - start)*1000, 4)))
            self.pub_detections.publish(msg)
            # self.pub_detections.publish(self.bridge.cv2_to_imgmsg(faces,"bgr8"))
        return self.toFaceDetectionMsg(boxes,probs)

    def toFaceDetectionMsg(self,face_locations=[],face_probs=[]):
        msg = GetFaceDetectionsResponse()
        if len(face_probs)>0 and len(face_probs) == len(face_locations):
            for (left, top, right, bottom), prob in zip(face_locations, face_probs):
                face_detection = FaceDetection("{}".format(round(prob, 2)),[left, top, right, bottom])
                msg.detections.append(face_detection)
        elif len(face_locations) > 0:
            for (left, top, right, bottom) in face_locations:
                face_detection = FaceDetection("-",[left, top, right, bottom])
                msg.detections.append(face_detection)
        return msg
    def cbSetThreshold(self,params):
        if params.value > 0 and params.value < 100:
            self.detector.setThreshold(params.value / 100.0)
        return SetInt32Response(params.port, params.value)

    def srvSetResize(self,params):
        if params.port <= 480 and params.port >= 10:
            if params.value <= 360 and params.value >= 10:
                self.detector.setResize(params.port,params.value)
        return SetInt32Response(params.port,params.value)
    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))
if __name__ == '__main__':
    rospy.init_node('ultra_face_inference_node', anonymous=False)
    node = UltraFaceInferenceNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()