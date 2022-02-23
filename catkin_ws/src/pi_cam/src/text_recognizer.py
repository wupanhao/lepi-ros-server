#!/usr/bin/python
#!coding:utf-8
import cv2
import numpy as np
import pytesseract
from PIL import Image, ImageFont, ImageDraw

defaultFont = ImageFont.truetype('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', 18)
def putText(frame, text, pos, color, font=defaultFont):
    """
    将文本显示在图片上
    Keyword arguments:
    frame: image 原图
    text: str 想要显示的文本
    pos: (x,y) 指定显示的初始坐标(左上角顶点)
    color: [r,g,b] 指定文本颜色的r、g、b值
    Returns:
    cv_img: image 叠加了文本的新图片(不改变原图)
    """
    pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_image)
    size = draw.textsize(text, font=font)
    draw.rectangle((pos[0], pos[1], pos[0]+size[0], pos[1]+size[1]), fill='#FFFFFF')
    draw.text(pos, text, font=font, fill=color)
    # draw.text(pos, text, fill=color)
    cv_img = cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)
    # cv2.rectangle(cv_img, (pos[0], pos[1]), (pos[0]+size[0], pos[1]+size[1]), (255, 255, 255), cv2.FILLED)
    return cv_img

class TextRecognizer(object):
    def __init__(self):
        self.roi = [(140,140),(340,220)]

    def detect(self,image,lang='chi_sim'):
        image_roi = image[self.roi[0][1]:self.roi[1][1],self.roi[0][0]:self.roi[1][0]]
        im = Image.fromarray(cv2.cvtColor(image_roi,cv2.COLOR_BGR2RGB))  
        return pytesseract.image_to_string(im,lang)

if __name__ == '__main__':
    import time
    import threading
    detector = TextRecognizer()
    running = True
    frame = None
    text = ''
    def capture_thread():
        global running
        global frame
        cap = cv2.VideoCapture(0)
        while True:
            # Grab a single frame of video
            ret, frame = cap.read()
            cv2.rectangle(frame,detector.roi[0],detector.roi[1],(10, 255, 0), 2)
            frame = putText(frame,text,detector.roi[0],(0, 0, 0))
            cv2.imshow("images", frame)
            if cv2.waitKey(2) == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
        running = False
    capture = threading.Thread(target = capture_thread)
    capture.start()
    while running == True:
        if frame is not None:
            start = time.time()
            text = detector.detect(frame)
            end = time.time()
            print("detect %d frame in %.2f ms" % (1, (end - start)*1000))

