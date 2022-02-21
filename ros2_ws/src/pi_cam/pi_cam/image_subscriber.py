#!/usr/bin/python
#!coding:utf-8
import rospy
from pi_cam.srv import GetFrame,GetFrameRequest
from cv_bridge import CvBridge

class RosImageProxy:
    def __init__(self):
        camera_frame = '/ubiquityrobot/camera_node/camera_get_frame'
        rospy.wait_for_service(camera_frame)
        self.bridge = CvBridge()
        self.get_frame = rospy.ServiceProxy(camera_frame, GetFrame)

class RosImageSubscriber:
    def __init__(self):
        from sensor_msgs.msg import Image
        import time
        image_topic = '/ubiquityrobot/camera_node/image_raw'
        rospy.init_node('image_subscriber', anonymous=True)
        self.cv_image = None
        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber(image_topic,Image, self.cbImg ,queue_size=1)
        while self.cv_image is None:
          print('wait for image msg')
          time.sleep(0.2)
    def cbImg(self,image_msg):
        if self.cv_image is None:
          print('image msg received')
        self.cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    def getImage(self):
      return self.cv_image
# print(__name__)
if __name__ == '__main__':
  import cv2
  import time
  from line_detector import LineDetector

  detector = LineDetector()

  subscriber = RosImageSubscriber()
  count = 1
  start = time.time()
  while True:
      # image_frame = subscriber.get_frame(GetFrameRequest())
      # cv_image = subscriber.bridge.imgmsg_to_cv2(image_frame.image, desired_encoding="bgr8")
      cv_image = subscriber.cv_image
      rect_image = cv_image

      detection,image_color = detector.detect_hsv(rect_image, detector.colors[u'黄色'])
      count = count + 1
      if(count > 100):
        end = time.time()
        print('detect %d frame in %.2f seconds, avarage %.1f ms/frame' % ( count, (end - start), (end - start)/count*1000 ))
        count = 0
        start = time.time()
      # if detection[0] != 0:
      #   print(detection)
      continue
      cv2.imshow('image',cv_image)
      c = cv2.waitKey(2)
      if c == 27:
        break
