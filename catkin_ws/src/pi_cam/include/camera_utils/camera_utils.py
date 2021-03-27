#!coding:utf-8
from subprocess import PIPE, Popen
import yaml
import cv2
import numpy as np
import os
from PIL import Image, ImageFont, ImageDraw
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import rospy

# from cam_info_reader_node
try:
    from sensor_msgs.msg import CameraInfo
except Exception as e:
    print(e)

    class CameraInfo:
        pass


def bgr_from_jpg(data):
    """ Returns an OpenCV BGR image from a string """
    s = np.fromstring(data, np.uint8)
    bgr = cv2.imdecode(s, cv2.IMREAD_COLOR)
    if bgr is None:
        msg = 'Could not decode image (cv2.imdecode returned None). '
        msg += 'This is usual a sign of data corruption.'
        raise ValueError(msg)
    return bgr


def jpg_from_bgr(image):
    return np.array(cv2.imencode('.jpg', image)[1]).tostring()


def load_camera_info_2(cali_file):
    with open(cali_file, 'r') as stream:
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info


def load_camera_info_3(cali_file="default.yaml"):
    # cali_file = os.path.dirname(os.path.abspath(__file__)) + "/../../camera_info/calibrations/" + cali_file
    cali_file = os.path.expanduser(
        '~')+"/Lepi_Data/ros/camera/calibrations/" + cali_file
    with open(cali_file, 'r') as stream:
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info


defaultFont = ImageFont.truetype(
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', 18)


def putText(frame, text, pos, color, font=defaultFont):
    """
    将文本显示在图片上
    Keyword arguments:
    frame: image 原图
    text：str 想要显示的文本
    pos：(x,y) 指定显示的初始坐标(左上角顶点)
    color: [r,g,b] 指定文本颜色的r、g、b值
    Returns:
    cv_img: image 叠加了文本的新图片(不改变原图)
    """
    pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_image)
    draw.text(pos, text.decode('utf-8'), font=font, fill=color)
    cv_img = cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)
    return cv_img


def putText3(frame, text, pos, color, font=defaultFont):
    pil_image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_image)
    size = draw.textsize(text, font=font)
    draw.rectangle((pos[0], pos[1], pos[0]+size[0],
                    pos[1]+size[1]), fill='#FFFFFF')
    draw.text(pos, text, font=font, fill=color)
    # draw.text(pos, text, fill=color)
    cv_img = cv2.cvtColor(np.asarray(pil_image), cv2.COLOR_RGB2BGR)
    # cv2.rectangle(cv_img, (pos[0], pos[1]), (pos[0]+size[0], pos[1]+size[1]), (255, 255, 255), cv2.FILLED)
    return cv_img


bridge = CvBridge()


def toImage(image_msg):
    if hasattr(image_msg, 'format'):  # CompressedImage
        try:
            cv_image = bgr_from_jpg(image_msg.data)
        except ValueError as e:
            print(e)
            return None
    else:  # Image
        try:
            cv_image = bridge.imgmsg_to_cv2(image_msg)
        except Exception as e:
            print(e)
            return None
    return cv_image


def toImageMsg(image):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = jpg_from_bgr(image)
    return msg


def cameraList(cam="Camera"):
    cmd = ["/usr/bin/v4l2-ctl", "--list-devices"]
    out, err = Popen(cmd, stdout=PIPE, stderr=PIPE).communicate()
    out = out.strip()
    # 将byte类型转换为string
    # out = str(out)
    # out = out.decode('utf-8')
    arr = []
    for dev in [i.split(b"\n\t") for i in out.split(b"\n\n")]:
        # print(dev)
        if len(dev) > 0 and ("Camera" in str(dev[0]) or "mmal" in str(dev[0])):
            for item in dev[1:]:
                cmd = ["/usr/bin/v4l2-ctl", "--list-formats", "-d", item]
                out, err = Popen(cmd, stdout=PIPE, stderr=PIPE).communicate()
                # print(len(out.split(b'\n')))
                if len(out.split(b'\n')) > 4:
                    arr.append(item.decode("utf-8"))
    return arr


def testPutText():
    import sys
    image = cv2.imread(sys.argv[1])
    image = putText3(image, u'手提箱: 53%', (50, 50), (0, 0, 0))
    cv2.imshow('Object detector', image)

    # Press any key to continue to next image, or press 'q' to quit
    if cv2.waitKey(0) == ord('q'):
        pass

    # Clean up
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print(cameraList())
