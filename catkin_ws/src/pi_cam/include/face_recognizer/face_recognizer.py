#!coding:utf-8
import face_recognition
import cv2
from PIL import Image,ImageFont,ImageDraw
import numpy as np
import os
import time

class FaceRecognizer(object):
    """docstring for FaceRecognizer"""
    def __init__(self,scale = 5,threshold = 0.45, fontSize = 18):
        super(FaceRecognizer, self).__init__()
        self.data_dir = "/home/pi/Data/face_recognizer/known_face"
        self.scale = scale
        self.font = ImageFont.truetype('/root/deps/msyh.ttc', fontSize)
        self.known_faces = {}
        self.threshold = threshold
        self.load_faces()
    def putText(self,frame, text, pos, color):
        pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil_image)
        draw.text(pos,text.decode('utf-8'),font=self.font,fill=color)
        cv_img = cv2.cvtColor(np.asarray(pil_image),cv2.COLOR_RGB2BGR)
        return cv_img

    def detect(self,frame,scale=None):
        if scale is None:
            scale = self.scale
        # Resize frame of video to 1/self.scale size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=1.0/scale, fy=1.0/scale)
        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]
        face_locations = face_recognition.face_locations(rgb_small_frame)
        return face_locations

    def recognize(self,frame,scale=None):
        if scale is None:
            scale = self.scale        
        small_frame = cv2.resize(frame, (0, 0), fx=1.0/scale, fy=1.0/scale)
        rgb_small_frame = small_frame[:, :, ::-1]
        face_locations = face_recognition.face_locations(rgb_small_frame)
        start = time.time()
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        face_names = []
        known_face_encodings = self.known_faces.values()
        known_face_names = self.known_faces.keys()
        for face_encoding in face_encodings:
            name = "未知"
            if len(known_face_encodings) > 0:
                # matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                # print(matches)
                print(face_distances)
                if face_distances[best_match_index] < self.threshold:
                    name = known_face_names[best_match_index]
            face_names.append(name)
        end = time.time()
        print("recognized %d faces in %.2f ms" % (len(face_locations),(end - start)*1000))            
        return face_locations,face_names
    def rect_faces(self,frame,face_locations,scale = None):
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
    def label_faces(self,frame,face_locations,face_names,scale = None):
        if scale is None:
            scale = self.scale  
        frame = self.rect_faces(frame,face_locations,scale)          
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
            frame = self.putText(frame, name, (left + 6, bottom - 24), color )
        return frame
    def load_faces(self):
        if not os.path.exists(self.data_dir):        
            os.makedirs(self.data_dir)        
        files = os.listdir(self.data_dir)
        for file in files:
            try:
                name = file.split('.')[0]
                file_path = os.path.join(self.data_dir,file)
                print(self.add_face_label(cv2.imread(file_path),name,scale = 1))
            except Exception as e:
                print(e)

    def add_face_label(self,frame,name,scale = None,save = False):
        if scale is None:
            scale = self.scale
        frame = cv2.resize(frame, (0, 0), fx=1.0/scale, fy=1.0/scale)
        rgb_frame = frame[:, :, ::-1]
        start = time.time()
        face_locations = self.detect(frame,scale = 1)
        end = time.time()
        print("Found %d faces in %.2f ms" % (len(face_locations),(end - start)*1000))

        if len(face_locations) == 1:
            face_encoding = face_recognition.face_encodings(frame,face_locations)[0]
            self.known_faces[name] = face_encoding
            if save:
                file_path = os.path.join(self.data_dir,name+'.png')
                print(file_path)
                cv2.imwrite(file_path,frame)
            return '成功添加"%s"的标记' % (name)
        elif len(face_locations) > 1:
            return '标记"%s"失败，检测到多余人脸' % (name)
        else:
            return '标记"%s"失败，未检测到人脸' % (name)
    def remove_face_label(self,name):
        if self.known_faces.has_key(name):
            self.known_faces.pop(name)
        try:
            os.system('rm '+os.path.join(self.data_dir,name+".*"))
            return "已删除"
        except Exception as e:
            print(e)
            return "删除出错"

def add_faces():
    fr = FaceRecognizer()
    frame = cv2.imread('./吴畔昊.png')
    print(fr.detect(frame))
    print(fr.add_face_label(frame,'畔昊',save = True))
    print(fr.add_face_label(cv2.imread('./ny.png'),'宁远',save = True))
    print(fr.add_face_label(cv2.imread('./obama.jpg'),'obama',save = True))
    face_locations,face_names = fr.recognize(frame)
    nframe = fr.label_faces(frame,face_locations,face_names)
    cv2.imshow('frame', nframe)
    cv2.waitKey(0)
def test_detect():
    fr = FaceRecognizer()
    cap = cv2.VideoCapture(0)
    while True:
        # Grab a single frame of video
        ret, frame = cap.read()
        start = time.time()
        face_locations = fr.detect(frame)
        nframe = fr.label_faces(frame,face_locations,face_names) 
        end = time.time()
        print("labeled %d faces in %.2f ms" % (len(face_locations),(end - start)*1000))           
        cv2.imshow("images", nframe)
        c = cv2.waitKey(4)
        if c == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
def test_recognize():
    fr = FaceRecognizer(scale = 5)
    print(fr.add_face_label(cv2.imread('./hq.png'),'海群',save = True))
    cap = cv2.VideoCapture(0)
    while True:
        # Grab a single frame of video
        ret, frame = cap.read()
        start = time.time()
        face_locations,face_names = fr.recognize(frame)
        nframe = fr.label_faces(frame,face_locations,face_names) 
        end = time.time()
        print("labeled %d faces in %.2f ms" % (len(face_locations),(end - start)*1000))           
        cv2.imshow("images", nframe)
        c = cv2.waitKey(4)
        if c == 27:
            break    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    test_recognize()