#!/usr/bin/python3
#!coding:utf-8
import time
# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M//g' camera_info_node.py
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
# from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
import numpy as np

from pi_cam.srv import GetPredictions, GetPredictionsResponse
# from pi_driver.srv import GetInt32,GetInt32Response
from pi_driver.srv import SetInt32, SetInt32Response
from pi_driver.srv import SetString, SetStringResponse, GetStrings, GetStringsResponse
from keras_transfer import ImageClassifier, AccuracyLogger
import os
from std_msgs.msg import String
from data_utils import get_labels
from pi_cam.srv import GetFrame, GetFrameRequest
from camera_utils import bgr_from_jpg, jpg_from_bgr


class TransferNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))

        self.IMAGE_W = 224
        self.IMAGE_H = 224
        self.bridge = CvBridge()
        self.ic = ImageClassifier()
        # self.ic.load_model('/root/keras/分类测试/model.h5')
        self.image_msg = None  # Image()
        self.getShm()
        self.training_logs_topic = rospy.Publisher(
            "~training_logs", String, queue_size=1)
        # self.pub_detections = rospy.Publisher("~image_transfer", CompressedImage, queue_size=1)

        rospy.Service('~camera_save_frame', SetString, self.srvCameraSaveFrame)
        rospy.Service('~set_ns', SetString, self.srv_set_ns)
        rospy.Service('~delete_ns', SetString, self.srv_delete_ns)
        rospy.Service('~create_cat', SetString, self.srv_create_cat)
        rospy.Service('~delete_cat', SetString, self.srv_delete_cat)
        rospy.Service('~list_ns', GetStrings, self.srv_list_ns)
        rospy.Service('~list_cat', GetStrings, self.srv_list_cat)
        rospy.Service('~train_classifier', SetString,
                      self.srv_train_classifier)
        rospy.Service('~predict', GetPredictions, self.srv_predict)
        rospy.Service('~get_training_data', GetPredictions,
                      self.srv_get_training_data)
        rospy.Service('~set_size', SetInt32, self.srv_set_size)
        # self.sub_image = rospy.Subscriber(
        #     "~image_raw/compressed", CompressedImage, self.cbImg,  queue_size=1)

        rospy.loginfo("[%s] Initialized......" % (self.node_name))

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
        xmin = int((480 - self.IMAGE_W)/2)
        ymin = int((360 - self.IMAGE_H)/2)
        xmax = xmin + self.IMAGE_W
        ymax = ymin + self.IMAGE_H
        return [xmin, ymin, xmax, ymax]

    def cbImg(self, image_msg):
        self.image_msg = image_msg
        return

    def srvCameraSaveFrame(self, params):
        if self.ic.ns is None:
            return SetStringResponse("训练没有定义,创建或者选择一次训练")
        try:
            cat_name = params.data
            directory = os.path.join(self.ic.data_root, self.ic.ns, cat_name)
            file_name = '%d.jpg' % (len(os.listdir(directory))+1)
            full_path = os.path.join(directory, file_name)
            cv_image = self.getImage()
            xmin, ymin, xmax, ymax = self.getBox()
            cv_image = cv_image[ymin:ymax, xmin:xmax]
            cv2.imwrite(full_path, cv_image)
            # print(cv_image)
            return SetStringResponse("保存至%s成功" % (full_path))
        except Exception as e:
            raise e
            return SetStringResponse("保存出错")
        finally:
            pass

    def srv_create_cat(self, params):
        if self.ic.ns == None:
            return SetStringResponse("请先设置训练名称")
        try:
            data_dir = os.path.join(self.ic.data_root, self.ic.ns, params.data)
            if os.path.exists(data_dir):
                print('data_dir %s exists' % (data_dir))
                return SetStringResponse("分类已存在")
            else:
                print('create data_dir %s' % (data_dir))
                os.makedirs(data_dir)
                return SetStringResponse("创建成功")
        except Exception as e:
            print(e)
            return SetStringResponse("创建失败")

    def srv_list_cat(self, params):
        if self.ic.ns is not None:
            data_dir = os.path.join(self.ic.data_root, self.ic.ns)
            dirs = []
            for item in os.listdir(data_dir):
                if os.path.isdir(os.path.join(data_dir, item)):
                    dirs.append(item)
            # print(dirs)
            return GetStringsResponse(dirs)
        else:
            return GetStringsResponse()

    def srv_list_ns(self, params):
        dirs = []
        for item in os.listdir(self.ic.data_root):
            if os.path.isdir(os.path.join(self.ic.data_root, item)):
                dirs.append(item)
        # print(dirs)
        return GetStringsResponse(dirs)

    def srv_set_ns(self, params):
        try:
            data_dir = os.path.join(self.ic.data_root, params.data)
            model_path = os.path.join(data_dir, 'model.h5')
            label_path = os.path.join(data_dir, 'labelmap.txt')
            self.ic.ns = params.data
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)
                return SetStringResponse("已创建训练,继续添加分类和数据以训练模型")
            if os.path.exists(model_path) and os.path.exists(label_path):
                self.ic.load_label_name()
                self.ic.load_model()
                return SetStringResponse("已加载训练,并加载已训练的模型")
            return SetStringResponse("已加载训练,添加分类和数据训练模型吧")
        except Exception as e:
            print(e)
            return SetStringResponse("设置失败")

    def srv_delete_ns(self, params):
        try:
            data_dir = os.path.join(self.ic.data_root, params.data)
            if len(data_dir) > len(self.ic.data_root):
                os.system('rm -rf '+data_dir)
                return SetStringResponse("删除成功")
        except Exception as e:
            print(e)
            return SetStringResponse("删除失败")

    def srv_delete_cat(self, params):
        if self.ic.ns is None:
            return SetStringResponse("请先设置训练名称")
        try:
            data_dir = os.path.join(self.ic.data_root, self.ic.ns, params.data)
            if len(data_dir) > len(self.ic.data_root):
                os.system('rm -rf '+data_dir)
                return SetStringResponse("删除成功")
        except Exception as e:
            print(e)
            return SetStringResponse("删除失败")

    def srv_train_classifier(self, params):
        try:
            epochs = int(params.data)
            if epochs <= 0:
                return SetStringResponse('至少训练1次')
            data_dir = os.path.join(self.ic.data_root, self.ic.ns)
            self.epochs = epochs
            self.ic.ns = self.ic.ns
            self.ic.train(data_dir, epochs, self.pub_training_logs)
            return SetStringResponse('训练完成,可以进行测试了')
        except Exception as e:
            print(e)
            raise e
            return SetStringResponse('训练失败')

    def srv_predict(self, params):
        try:
            cv_image = self.getImage()
            xmin, ymin, xmax, ymax = self.getBox()
            cv_image = cv_image[ymin:ymax, xmin:xmax]
            res = self.ic.predict(cv_img=cv_image)
            if res is not None:
                return GetPredictionsResponse(res[0], res[1])
        except Exception as e:
            print(e)
            return GetPredictionsResponse()

    def pub_training_logs(self, epoch, batch, logs):
        # logs {'loss': 0.33773628, 'accuracy': 0.71428573, 'batch': 6, 'size': 4}
        print(logs)
        msg = String('第%d/%d轮, 批次: %d, 损失: %.2f, 准确率: %.2f' %
                     (epoch+1, self.epochs, batch, logs['loss'], logs['accuracy']))
        self.training_logs_topic.publish(msg)

    def srv_get_training_data(self, params):
        data_dir = os.path.join(self.ic.data_root, self.ic.ns)
        labels = get_labels(data_dir)
        counts = [len(os.listdir(os.path.join(data_dir, label)))
                  for label in labels]
        return GetPredictionsResponse(labels, counts)

    def srv_set_size(self, params):
        if params.port <= 480 and params.port >= 10:
            self.IMAGE_W = params.port
        if params.value <= 360 and params.value >= 10:
            self.IMAGE_H = params.value
        return SetInt32Response(params.port, params.value)

    def pubImage(self, image):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = jpg_from_bgr(image)
        self.pub_detections.publish(msg)

    def onShutdown(self):
        self.shm.close()
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('transfer_learning_node', anonymous=False)
    node = TransferNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
