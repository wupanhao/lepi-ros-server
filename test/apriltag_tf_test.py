#!/usr/bin/env python
#!coding:utf-8
import math
import tf
import rospy
import roslib
# roslib.load_manifest('learning_tf')
from transforms3d.euler import euler2quat
from apriltag_detector import ApriltagDetector
from camera_utils import cameraList
import cv2
import numpy as np


def sendTransform():
    rospy.init_node('dynamic_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() * math.pi
        br.sendTransform((2.0 * math.sin(t), 2.0 * math.cos(t), 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "carrot1",
                         "map")
        br.sendTransform((-0.5 * math.sin(t), 0.0, -0.5 * math.cos(t)),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "carrot2",
                         "carrot1")
        rospy.sleep(1/10.0)


if __name__ == '__main__':
    import time
    detector = ApriltagDetector()
    # import os
    # os.system(
    #     'wget https://april.eecs.umich.edu/media/apriltag/apriltagrobots_overlay.jpg -O /tmp/test.jpg')
    # test_image = cv2.imread('/tmp/test.jpg')
    # print(detector.detect(test_image))
    dev = cameraList()
    cap = cv2.VideoCapture(dev[0])
    rospy.init_node('dynamic_tf_broadcaster')
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        # Grab a single frame of video
        ret, frame = cap.read()
        # 不能水平镜像翻转，tag会变
        # frame = cv2.flip(frame, 1)
        start = time.time()
        detector.detect(frame)
        detections = detector.toApriltagDetections()
        for i, tag in enumerate(detections):
            print(i, tag)
            euler = np.array(tag[1])/180.0*math.pi
            pos = np.array(tag[2])/100.0
            br.sendTransform((pos[2], -pos[0], -pos[1]),
                             euler2quat(-euler[1], euler[0]+np.pi, euler[2]),
                             rospy.Time.now(),
                             "carrot1",
                             "map")
        end = time.time()
        # detector.label_tags(frame, tags)
        print("detect %d frame in %.2f ms" %
              (1, (end - start)*1000))
        cv2.imshow("images", frame)
        c = cv2.waitKey(4)
        if c == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
