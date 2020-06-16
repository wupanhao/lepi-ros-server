#!coding:utf-8
import cv2
import threading
import time
# from .image_rector import ImageRector
import image_geometry
from camera_utils import load_camera_info_3

class UsbCamera(object):
    """docstring for UsbCamera"""
    def __init__(self, rate=30,callback=None):
        super(UsbCamera, self).__init__()
        self.cap = None
        self.rate = rate
        self._reader = None #threading.Thread(target=self.continuous_capture)
        self.last_image = None
        self.callback = callback
        self.rectify = False
        self.flip_code = 2
        self.active = False
        # self.rector = ImageRector()
        self.cameraModel = image_geometry.PinholeCameraModel()
        self.loadCaliFile()
    def open_camera(self,camera_id=0):
        # self.rector = ImageRector(cali_file="default.yaml")
        # self.rector = ImageRector(size=(480,360),cali_file="default.yaml")
        self.camera_id = camera_id
        try:
            if (self._reader is not None) and( self._reader.isAlive()):
                print('thread is active , stop it')
                self.active = False
                time.sleep(0.5)
            self._reader = threading.Thread(target=self.continuous_capture)
            if self.cap is not None:
                self.cap.release()
                self.cap = None
                # cv2.CAP_OPENCV_MJPEG
            self.cap = cv2.VideoCapture(camera_id)
            if self.cap.isOpened():
                self.active = True
                self.cap.set(cv2.CAP_PROP_FPS,30)
                print('open camera with index %d successfully' % (camera_id) )
                self._reader.start()
                while self.last_image is None:
                    time.sleep(0.5)
                    print('wait for first image')
                print('first image captured')
                return 0
            else:
                print('open camera with index %d failed' % (camera_id) )
                self.cap = cv2.VideoCapture(camera_id+1)
            if self.cap.isOpened():
                self.active = True
                print('open camera with index %d successfully' % (camera_id+1) )
                self._reader.start()
                return 0
            else:
                print('open camera with index %d failed' % (camera_id+1) )
                self.active = False
                return 1
        except Exception as e:
            print('open camera with index %d failed' % (camera_id) )
            print(e)
            return 1
        finally:
            pass
    def close_camera(self):
        print('close_camera',self.active,self.cap)
        if self.cap is not None:
            try:
                self.cap.release()
                cv2.destroyAllWindows() 
            except Exception as e:
                print(e)
            finally:
                self.cap = None
    def continuous_capture(self):
        if (self.cap is None) or (not self.cap.isOpened()):
            print('camera not opened')
            return
        print('start continuous_capture thread')
        while self.active == True and self.cap is not None:
            try:
                ret,frame = self.cap.read()
                if ret == True:
                    self.last_image = frame
                    if self.callback is not None :
                        self.callback(frame)
                else:
                    # self.last_image = None
                    pass
            except Exception as e:
                raise e
                print('camera capture error',e)
                time.sleep(1)
            finally:
                #time.sleep(1.0/self.rate)
                time.sleep(0.005)
        self.close_camera()
    def save_a_frame(self,full_path):
        if self.last_image is not None:
            try:
                cv2.imwrite(full_path,self.last_image)
                print('saved image to '+full_path)
                return 0
            except Exception as e:
                print(e)
                return 1
        else:
            return 2
    def getImage(self):
        if self.last_image is None:
            return None
        cv_image = self.last_image
        # cv_image = cv2.resize(cv_image,(480,360))
        if self.rectify:
            self.cameraModel.rectifyImage(cv_image,cv_image)
            # cv_image = self.rector.rect(cv_image)
        # cv_image = cv2.resize(cv_image,(480,360))
        if abs(self.flip_code) <= 1:
            cv_image = cv2.flip(cv_image,self.flip_code)
        cv_image = cv2.resize(cv_image,(480,360))
        return cv_image
    def setFlip(self,flip_code=2):
        self.flip_code = flip_code
    def setRectify(self,rectify=False):
        self.rectify = rectify
    def loadCaliFile(self,cali_file="default.yaml"):
        self.camera_info_msg = load_camera_info_3(cali_file)
        self.cameraModel.fromCameraInfo(self.camera_info_msg)
def show_pic(frame):
    cv2.imshow("capture", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      print('q pressed')
if __name__ == '__main__':
    cam = UsbCamera(callback=show_pic)
    cam.open_camera()
    time.sleep(20)
    cam.active = False
    time.sleep(3)

    cam.open_camera()
    #cam.open_camera(1)
    time.sleep(10)
    cam.active = False
    time.sleep(3)
