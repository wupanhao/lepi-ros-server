#!/usr/bin/python3
#!coding:utf-8
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image,CompressedImage
from camera_utils import bgr_from_jpg,jpg_from_bgr,putText3
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import ObjectDetection
from pi_cam.srv import GetObjectDetections,GetObjectDetectionsResponse

from pyzbar import pyzbar

class BarcodeScannerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.bridge = CvBridge()
        self.visualization = True
        self.image_msg = None
        self.pub_detections = rospy.Publisher("~image_barcode", CompressedImage, queue_size=1)

        rospy.Service('~barcode_scan', GetObjectDetections, self.cbBarcodeScan)

        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        self.sub_image = rospy.Subscriber("~image_raw/compressed", CompressedImage, self.cbImg ,  queue_size=1)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbImg(self,image_msg):
        self.image_msg = image_msg
        # cv_image = self.getImage(image_msg)
        # cv2.rectangle(cv_image,self.roi[0],self.roi[1],(10, 255, 0), 2)
        # self.pubImage(cv_image)

    def detect(self,frame):
        return pyzbar.decode(frame)

    def drawLabel(self,frame,barcodes):
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type

            text = barcode.data.decode("utf-8")
            # print(text,text.decode("utf-8"))
            # text = "{} ({})".format(barcodeData, barcodeType)
            # cv2.putText(frame, text, (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            frame = putText3(frame, text, (x, y - 10),(0, 0, 255))
        return frame

    def cbBarcodeScan(self,params):
        image = self.getImage(self.image_msg)
        barcodes = self.detect(image)
        # print(barcodes)
        self.pubImage(self.drawLabel(image,barcodes))
        return self.toBarcodeDetection(barcodes)

    def toBarcodeDetection(self,barcodes):
        res = GetObjectDetectionsResponse()
        for barcode in barcodes:
            # (x, y, w, h) = barcode.rect
            msg = ObjectDetection(barcode.rect,barcode.data.decode("utf-8"),0)
            res.detections.append(msg)
        return res

    def pubImage(self,image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = jpg_from_bgr(image)
        self.pub_detections.publish(msg)   
    def getImage(self,image_msg):
        if hasattr(image_msg,'format'): # CompressedImage
            try:
                cv_image = bgr_from_jpg(image_msg.data)
            except ValueError as e:
                rospy.loginfo('[%s] cannot decode image: %s' % (self.node_name,e))
                return None
        else: # Image
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg)
            except Exception as e:
                rospy.loginfo('[%s] cannot convert image: %s' % (self.node_name,e))
                return None
        return cv_image
    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
    rospy.init_node('barcode_scanner_node', anonymous=False)
    node = BarcodeScannerNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()



