#!/usr/bin/python3
#!coding:utf-8
import cv2
from pyee import ExecutorEventEmitter
import rospy
from sensor_msgs.msg import CompressedImage
from camera_utils import toImageMsg, putText3
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from pi_cam.msg import ObjectDetection
from pi_cam.srv import GetObjectDetections, GetObjectDetectionsResponse
from pyzbar import pyzbar


class BarcodeScannerNode(ExecutorEventEmitter):
    def __init__(self):
        super().__init__()
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.visualization = True
        self.image_msg = None
        self.getShm()
        self.on('pub_image', self.pubImage)
        self.pub_detections = rospy.Publisher(
            "~image_barcode", CompressedImage, queue_size=1)
        rospy.Service('~barcode_scan', GetObjectDetections, self.cbBarcodeScan)
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
        rect_image = self.image_frame.copy()
        return cv2.resize(rect_image, (480, 360))


    def detect(self, frame):
        return pyzbar.decode(frame)

    def drawLabel(self, frame, barcodes):
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            text = barcode.data.decode("utf-8")
            # print(text,text.decode("utf-8"))
            # text = "{} ({})".format(text, barcode.type)
            # cv2.putText(frame, text, (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            frame = putText3(frame, text, (x, y - 10), (0, 0, 255))
        return frame

    def cbBarcodeScan(self, params):
        self.image = self.getImage()
        self.barcodes = self.detect(self.image)
        # print(barcodes)
        if self.visualization:
            self.emit('pub_image')
        return self.toBarcodeDetection(self.barcodes)

    def toBarcodeDetection(self, barcodes):
        res = GetObjectDetectionsResponse()
        for barcode in barcodes:
            # (x, y, w, h) = barcode.rect
            msg = ObjectDetection(
                barcode.rect, barcode.data.decode("utf-8"), 0)
            res.detections.append(msg)
        return res

    def pubImage(self):
        image = self.image
        barcodes = self.barcodes
        msg = toImageMsg(self.drawLabel(image, barcodes))
        self.pub_detections.publish(msg)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('barcode_scanner_node', anonymous=False)
    node = BarcodeScannerNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
