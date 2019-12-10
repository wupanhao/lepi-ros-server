#!coding:utf-8
import cv2
import threading
import time

class UsbCamera(object):
	"""docstring for UsbCamera"""
	def __init__(self, rate=20,callback=None):
		super(UsbCamera, self).__init__()
		self.cap = None
		self.rate = 20
		self._reader = None #threading.Thread(target=self.continuous_capture)
		self.last_image = None
		self.callback = callback
		self.active = False
	def open_camera(self,camera_id=0):
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
			self.cap = cv2.VideoCapture(camera_id)
			if self.cap.isOpened():
				self.active = True
				self._reader.start()
				print('open camera with index %d successfully' % (camera_id) )
				return 0
			else:
				print('open camera with index %d failed' % (camera_id) )
				self.cap = cv2.VideoCapture(camera_id+1)
			if self.cap.isOpened():
				self.active = True
				self._reader.start()
				print('open camera with index %d successfully' % (camera_id+1) )
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
		print('close_camera')
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
		while self.active == True:
			try:
				ret,frame = self.cap.read()
				if ret == True:
					self.last_image = frame
					if (frame is not None) and (self.callback is not None) :
						self.callback(frame)
				else:
					self.last_image = None
					pass
			except Exception as e:
				# raise e
				print('camera capture error',e)
				time.sleep(1)
			finally:
				time.sleep(1.0/self.rate)
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