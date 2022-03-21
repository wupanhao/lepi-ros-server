#!/usr/bin/python3
#!coding:utf-8
import cv2
from PIL import Image, ImageFont, ImageDraw
import numpy as np
from paddleocr import PaddleOCR
defaultFont = ImageFont.truetype(
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', 48)


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
    if hasattr(text, "decode"):
        text = text.decode('utf-8')
    pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_image)
    draw.text(pos, text, font=font, fill=color)
    cv_img = cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)
    return cv_img


class TextRecognizer(object):
    def __init__(self):
        self.roi = [(160, 160), (480, 320)]
        # 支持 `ch`, `en`, `fr`, `german`, `korean`, `japan`
        # --use_angle_cls true设置使用方向分类器识别180度旋转文字
        self.ocr = PaddleOCR(use_angle_cls=False, lang='ch')

    def detect(self, image, lang=None):
        img = image[self.roi[0][1]:self.roi[1][1],
                    self.roi[0][0]:self.roi[1][0]]
        result = self.ocr.ocr(img, cls=False)
        print(result)
        text = ''
        for line in result:
            text = text+line[1][0]+' '
        return text

    def draw_result(self, image, text):
        cv2.rectangle(image, self.roi[0],
                      self.roi[1], (0, 255, 0), 4)
        if len(text) > 0:
            color = (0, 0, 255)
            frame = putText(
                image, text, (recognizer.roi[0][0] + 6, recognizer.roi[1][1]), color)
            return frame
        return image


if __name__ == '__main__':
    recognizer = TextRecognizer()
    Key_Esc = 27

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        # 逐帧捕获
        ret, frame = cap.read()
        text = recognizer.detect(frame)
        frame = recognizer.draw_result(frame, text)
        # 显示原图或灰度图
        cv2.imshow('frame', np.rot90(cv2.resize(frame, (320, 240))))
        # 按Esc键退出
        if cv2.waitKey(1) == Key_Esc:
            break
    # 完成所有操作后，释放捕获器
    cap.release()
    cv2.destroyAllWindows()
