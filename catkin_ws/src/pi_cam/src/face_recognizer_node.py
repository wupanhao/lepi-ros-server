#!/usr/bin/python
#!coding:utf-8
# import the necessary packages
import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import FaceDetection
from pi_cam.srv import GetFaceDetections, GetFaceDetectionsResponse
from pi_driver.srv import GetStrings, GetStringsResponse, SetString, SetStringResponse

from camera_utils import toImage, toImageMsg
from face_recognizer import FaceRecognizer

# (0,10) => (-320,230)
# (320,240) => (0,0)


def toScratchAxes(cv_x, cv_y):  # 480x360 in Scratch
    return (cv_x-240, cv_y*-1+180)


class FaceRecognizerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.visualization = True

        self.image_msg = None
        self.getShm()
        self.pub_detections = rospy.Publisher(
            "~image_face", CompressedImage, queue_size=1)
        self.recognizer = FaceRecognizer(scale=4)

        rospy.Service('~detect_face_locations',
                      GetFaceDetections, self.cbDetectFaceLocations)
        rospy.Service('~detect_face_labels', GetFaceDetections,
                      self.cbDetectFaceLabels)
        rospy.Service('~list_face_labels', GetStrings, self.cbListFaceLabels)
        rospy.Service('~add_face_label', SetString, self.cbAddFaceLabel)
        rospy.Service('~remove_face_label', SetString, self.cbRemoveFaceLabel)
        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg , queue_size=1)
        # self.sub_image = rospy.Subscriber(
        #     "~image_raw/compressed", CompressedImage, self.cbImg,  queue_size=1)

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def getShm(self):
        from pi_driver import SharedMemory
        import time
        import numpy as np
        while True:
            try:
                self.shm = SharedMemory('cv_image')
                self.image_frame = np.ndarray(
                    (480, 640, 3), dtype=np.uint8, buffer=self.shm.buf)
                break
            except:
                print(self.node_name, 'wait for SharedMemory cv_image')
                time.sleep(1)

    def getImage(self):
        import cv2
        rect_image = self.image_frame.copy()
        return cv2.resize(rect_image, (480, 360))

    def cbImg(self, image_msg):
        self.image_msg = image_msg

    def cbDetectFaceLocations(self, params):
        rect_image = self.getImage()
        faces = self.recognizer.detect(rect_image)
        if self.visualization:
            self.pubImage(faces)
        return self.toFaceDetectionMsg(self.recognizer.face_locations)

    def cbDetectFaceLabels(self, params):
        rect_image = self.getImage()
        faces = self.recognizer.recognize(rect_image)
        if self.visualization:
            self.pubImage(faces)
        return self.toFaceDetectionMsg(self.recognizer.face_locations, self.recognizer.face_names)

    def toFaceDetectionMsg(self, face_locations=[], face_names=[]):
        msg = GetFaceDetectionsResponse()
        scale = self.recognizer.scale
        if len(face_names) > 0 and len(face_names) == len(face_locations):
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                face_detection = FaceDetection(
                    name, [left*scale, top*scale, right*scale, bottom*scale])
                msg.detections.append(face_detection)
        elif len(face_locations) > 0:
            for (top, right, bottom, left) in face_locations:
                face_detection = FaceDetection(
                    "", [left*scale, top*scale, right*scale, bottom*scale])
                msg.detections.append(face_detection)
        return msg

    def cbListFaceLabels(self, params):
        return GetStringsResponse(self.recognizer.known_faces.keys())

    def cbAddFaceLabel(self, params):
        if len(params.data) == 0:
            return SetStringResponse("添加失败,名称长度为0")
        try:
            cv_image = toImage(self.image_msg)
            res = self.recognizer.add_face_label(
                cv_image, params.data, save=True)
            return SetStringResponse(res)
        except Exception as e:
            print(e)
            return SetStringResponse("添加失败")

    def cbRemoveFaceLabel(self, params):
        if len(params.data) == 0:
            return SetStringResponse("未提供要删除的标签")
        try:
            res = self.recognizer.remove_face_label(params.data)
            return SetStringResponse(res)
        except Exception as e:
            print(e)
            return SetStringResponse("删除失败")

    def pubImage(self, image):
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('face_recognizer_node', anonymous=False)
    node = FaceRecognizerNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
