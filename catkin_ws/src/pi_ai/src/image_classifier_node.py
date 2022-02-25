#!/usr/bin/python3
#!coding:utf-8
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import jpg_from_bgr
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np
from pi_cam.msg import ObjectDetection
from pi_cam.srv import GetObjectDetections,GetObjectDetectionsResponse
from pi_driver.srv import SetInt32,SetInt32Response
from pi_ai import ImageClassifier

class ImageClassifierNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.IMAGE_W = 224
        self.IMAGE_H = 224
        self.visualization = True
        self.image_msg = None
        # self.pub_detections = rospy.Publisher("~image_class", CompressedImage, queue_size=1)
        self.getShm()
        rospy.Service('~class_image', GetObjectDetections, self.cbGetObjectDetections)
        rospy.Service('~set_threshold', SetInt32, self.cbSetThreshold)
        rospy.Service('~set_size', SetInt32, self.srvSetSize)
        self.detector = ImageClassifier()
        try:
            self.detector.load_model(use_TPU=True)
        except Exception as e:
            print(e)
            self.detector.load_model(use_TPU=False)
        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        self.sub_image = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.cbImg ,  queue_size=1)

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


    def getBox(self):
        xmin = (480 - self.IMAGE_W)//2
        ymin = (360 - self.IMAGE_H)//2
        xmax = xmin + self.IMAGE_W
        ymax = ymin + self.IMAGE_H
        return [xmin,ymin,xmax,ymax]
    def cbImg(self,image_msg):
        self.image_msg = image_msg
        return

    def cbGetObjectDetections(self,params):
        image = self.getImage()
        xmin,ymin,xmax,ymax = self.getBox()
        image = image[ymin:ymax,xmin:xmax]
        detections = self.detector.detect(image)
        # print(detections)
        return self.toObjectDetections(detections)

    def toObjectDetections(self,detections):
        rsp = GetObjectDetectionsResponse()
        for class_,score in detections:
            if score >= self.detector.min_conf_threshold:
                detection = ObjectDetection([0,0,0,0], class_,score*100)
                rsp.detections.append(detection)
        return rsp
    def cbSetThreshold(self,params):
        self.detector.set_threshold(params.value)
        return SetInt32Response(params.port, params.value)

    def srvSetSize(self,params):
        if params.port <= 480 and params.port >= 10:
            self.IMAGE_W = params.port
        if params.value <= 360 and params.value >= 10:
            self.IMAGE_H = params.value
        return SetInt32Response(params.port,params.value)
    def pubImage(self,image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = jpg_from_bgr(image)
        self.pub_detections.publish(msg)   

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
    rospy.init_node('image_classifier_node', anonymous=False)
    node = ImageClassifierNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
