#!/usr/bin/python
#!coding:utf-8
import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from camera_utils import toImageMsg, toImage
from image_processor import ImageProcessor
from pi_driver.srv import GetString, GetStringResponse
from pi_cam.srv import AddProc, AddProcResponse


class ImageProcessorNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.visualization = True
        self.image_msg = None
        self.processor = ImageProcessor()
        self.pub_detections = rospy.Publisher(
            "~image_processed", CompressedImage, queue_size=1)

        rospy.Service('~add_proc', AddProc, self.cbAddProc)
        rospy.Service('~exec_proc', GetString, self.cbExecProc)

        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
        self.sub_image = rospy.Subscriber(
            "~image_raw/compressed", CompressedImage, self.cbImg,  queue_size=1)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbImg(self, image_msg):
        self.image_msg = image_msg
        return

    def cbAddProc(self, params):
        self.processor.addProc(params.name, params.args)
        return AddProcResponse("添加成功")

    def cbExecProc(self, params):
        image = toImage(self.image_msg)
        img = self.processor.process(image)
        self.pubImage(img)
        return GetStringResponse('执行成功')

    def pubImage(self, image):
        msg = toImageMsg(image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('image_processor_node', anonymous=False)
    node = ImageProcessorNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
