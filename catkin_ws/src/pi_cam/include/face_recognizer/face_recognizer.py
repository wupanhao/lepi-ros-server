#!coding:utf-8
import face_recognition
import cv2
# from PIL import Image, ImageFont, ImageDraw
import numpy as np
import os
import time

from camera_utils import putText

class FaceRecognizer(object):
    """
    FaceRecognizer类, 用来检测和识别人脸
    Attributes:
        data_dir: str 标记人脸图片的保存目录
        scale: int 缩放倍数，加快检测速度但会降低检测精度
        font: ImageFont 中文字体
        known_faces：dict 存放已标记人脸的字典
        threshold：float 检测阈值，小于该值才会进行相似度比较
    """

    def __init__(self, scale=5, threshold=0.45, fontSize=18):
        super(FaceRecognizer, self).__init__()
        self.data_dir = "/home/pi/Lepi_Data/ros/face_recognizer/known_face"
        self.scale = scale
        # self.font = ImageFont.truetype(
        #     '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf', fontSize)
        self.known_faces = {}
        self.threshold = threshold
        self.load_faces()


    def detect(self, frame, scale=None):
        """
        人脸检测函数
        Keyword arguments:
        frame: image 原图
        scale：float 缩放倍数
        Returns:
        face_locations: [(x1,y1,x2,y2)] 表示所有检测到人脸的位置坐标数组，每个数组元素代表一个人脸的位置
        """
        if scale is None:
            scale = self.scale
        # Resize frame of video to 1/self.scale size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=1.0/scale, fy=1.0/scale)
        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]
        face_locations = face_recognition.face_locations(rgb_small_frame)
        return face_locations

    def recognize(self, frame, scale=None):
        """
        人脸识别函数
        Keyword arguments:
        frame: image 原图
        scale：float 缩放倍数
        Returns:
        face_locations: [(x1,y1,x2,y2)] 表示所有检测到人脸的位置坐标数组，每个数组元素代表一个人脸的位置
        face_names: ['未知'] 对应每个人脸的标签，未检测到用'未知'表示
        """
        if scale is None:
            scale = self.scale
        small_frame = cv2.resize(frame, (0, 0), fx=1.0/scale, fy=1.0/scale)
        rgb_small_frame = small_frame[:, :, ::-1]
        face_locations = face_recognition.face_locations(rgb_small_frame)
        start = time.time()
        face_encodings = face_recognition.face_encodings(
            rgb_small_frame, face_locations)
        face_names = []
        known_face_encodings = self.known_faces.values()
        known_face_names = self.known_faces.keys()
        for face_encoding in face_encodings:
            name = "未知"
            if len(known_face_encodings) > 0:
                # matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                face_distances = face_recognition.face_distance(
                    known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                # print(matches)
                print(face_distances)
                if face_distances[best_match_index] < self.threshold:
                    name = known_face_names[best_match_index]
            face_names.append(name)
        end = time.time()
        print("recognized %d faces in %.2f ms" %
              (len(face_locations), (end - start)*1000))
        return face_locations, face_names

    def rect_faces(self, frame, face_locations, scale=None):
        """
        把检测到的人脸用矩形框出来
        Keyword arguments:
        frame: image 原图
        face_locations: [(x1,y1,x2,y2)] 表示所有检测到人脸的位置坐标数组，每个数组元素代表一个人脸的位置
        scale：float 缩放倍数
        Returns:
        frame: image 框出了人脸的新图像(改变原图)
        """
        if scale is None:
            scale = self.scale
        # Display the results
        for (top, right, bottom, left) in face_locations:
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= scale
            right *= scale
            bottom *= scale
            left *= scale
            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
            # Draw a label with a name below the face
        return frame

    def label_faces(self, frame, face_locations, face_names, scale=None):
        """
        给识别到的人脸添加标签
        Keyword arguments:
        frame: image 原图
        face_locations: [(x1,y1,x2,y2)] 表示所有检测到人脸的位置坐标数组，每个数组元素代表一个人脸的位置
        scale：float 缩放倍数，需要和检测时的一致，否则比例会出错
        Returns:
        frame: image 标记了人脸的新图像(不改变原图)
        """
        if scale is None:
            scale = self.scale
        frame = self.rect_faces(frame, face_locations, scale)
        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= scale
            right *= scale
            bottom *= scale
            left *= scale
            # Draw a box around the face
            # cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
            # Draw a label with a name below the face
            color = (0, 0, 255)
            frame = putText(frame, name, (left + 6, bottom - 24), color)
        return frame

    def load_faces(self):
        """
        加载本地的人脸标签(默认存放在data_dir下)
        Keyword arguments:
        无
        Returns:
        无
        """
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        files = os.listdir(self.data_dir)
        for file in files:
            try:
                name = file.split('.')[0]
                file_path = os.path.join(self.data_dir, file)
                print(self.add_face_label(cv2.imread(file_path), name, scale=1))
            except Exception as e:
                print(e)

    def add_face_label(self, frame, name, scale=None, save=False):
        """
        动态添加人脸标签
        Keyword arguments:
        frame：image 原图
        name：str 标签名称
        scale：float 缩放倍数
        save：bool 是否保存，True则将保存图片至本地标签目录，每次启动会重新读取，False只在本次运行生效
        Returns:
        无
        """
        if scale is None:
            scale = self.scale
        frame = cv2.resize(frame, (0, 0), fx=1.0/scale, fy=1.0/scale)
        rgb_frame = frame[:, :, ::-1]
        start = time.time()
        face_locations = self.detect(frame, scale=1)
        end = time.time()
        print("Found %d faces in %.2f ms" %
              (len(face_locations), (end - start)*1000))

        if len(face_locations) == 1:
            face_encoding = face_recognition.face_encodings(
                frame, face_locations)[0]
            self.known_faces[name] = face_encoding
            if save:
                file_path = os.path.join(self.data_dir, name+'.png')
                print(file_path)
                cv2.imwrite(file_path, frame)
            return '成功添加"%s"的标记' % (name)
        elif len(face_locations) > 1:
            return '标记"%s"失败，检测到多余人脸' % (name)
        else:
            return '标记"%s"失败，未检测到人脸' % (name)

    def remove_face_label(self, name):
        """
        删除人脸标签
        Keyword arguments:
        name：str 标签名称
        Returns:
        无
        """
        if self.known_faces.has_key(name):
            self.known_faces.pop(name)
        try:
            os.system('rm '+os.path.join(self.data_dir, name+".*"))
            return "已删除"
        except Exception as e:
            print(e)
            return "删除出错"

def add_faces():
    """
    测试函数，添加标签并识别
    Keyword arguments:
    无
    Returns:
    无
    """
    fr = FaceRecognizer()
    frame = cv2.imread('./吴畔昊.png')
    print(fr.detect(frame))
    print(fr.add_face_label(frame, '畔昊', save=True))
    print(fr.add_face_label(cv2.imread('./ny.png'), '宁远', save=True))
    print(fr.add_face_label(cv2.imread('./obama.jpg'), 'obama', save=True))
    face_locations, face_names = fr.recognize(frame)
    nframe = fr.label_faces(frame, face_locations, face_names)
    cv2.imshow('frame', nframe)
    cv2.waitKey(0)


def test_detect():
    """
    测试函数，打开摄像头实时检测人脸, 按Esc退出
    Keyword arguments:
    无
    Returns:
    无
    """
    fr = FaceRecognizer()
    cap = cv2.VideoCapture(0)
    while True:
        # Grab a single frame of video
        ret, frame = cap.read()
        start = time.time()
        face_locations = fr.detect(frame)
        nframe = fr.rect_faces(frame, face_locations)
        end = time.time()
        print("labeled %d faces in %.2f ms" %
              (len(face_locations), (end - start)*1000))
        cv2.imshow("images", nframe)
        c = cv2.waitKey(4)
        if c == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


def test_recognize():
    """
    测试函数，打开摄像头实时识别人脸, 按Esc退出
    Keyword arguments:
    无
    Returns:
    无
    """
    fr = FaceRecognizer(scale=5)
    print(fr.add_face_label(cv2.imread('./hq.png'), '海群', save=True))
    cap = cv2.VideoCapture(0)
    while True:
        # Grab a single frame of video
        ret, frame = cap.read()
        start = time.time()
        face_locations, face_names = fr.recognize(frame)
        nframe = fr.label_faces(frame, face_locations, face_names)
        end = time.time()
        print("labeled %d faces in %.2f ms" %
              (len(face_locations), (end - start)*1000))
        cv2.imshow("images", nframe)
        c = cv2.waitKey(4)
        if c == 27:
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # test_recognize()
    test_detect()
    #fr = FaceRecognizer()
    #frame = cv2.imread('./吴畔昊.png')
    #print(fr.detect(frame))
    #print(fr.add_face_label(frame, 'panhao', save=True))
