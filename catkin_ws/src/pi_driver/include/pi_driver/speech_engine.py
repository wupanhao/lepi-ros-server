#!coding:utf-8
import os

class SpeetchEngine:
	def __init__(self):
		#语言:中文
		self.voice = "zh"
		#音量大小
		self.amplification = 50
		#语调
		self.pitch = 80
		#语速
		self.speed = 200
	def to_speetch(self,text=None):
		if text is not None:
			os.system('espeak "%s" -v %s -a %d -p %d -s %d' %(text,self.voice,self.amplification,self.pitch,self.speed))
	def keyword_spot():
		pass

if __name__ == '__main__':
	se = SpeetchEngine()
	se.to_speetch(123)
	se.to_speetch('你好')
	se.to_speetch('1.23')