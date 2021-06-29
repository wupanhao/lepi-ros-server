#!/usr/bin/python3
#!coding:utf-8
import os
import threading
import re
import sys
import time
import rospy
from pi_driver.srv import SetString, SetStringResponse
from pi_driver.srv import GetCommandDetection, GetCommandDetectionResponse
from std_msgs.msg import String

root_dir = os.path.expanduser(
    '~')+'/Lepi_Data/ros/smart_audio_node'
sys.path.append(root_dir)
if True:
    import snowboydecoder


class SmartAudioNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.bin_dir = root_dir
        # self.pub_detections = rospy.Publisher("~image_text", CompressedImage, queue_size=1)
        self.hotwordDetect = False

        keyword_model = self.bin_dir+'/resources/models/snowboy.umdl'
        self.detector = snowboydecoder.HotwordDetector(
            keyword_model, sensitivity=0.5)
        self.pubHotwordDetect = rospy.Publisher(
            '~hotword_detect', String, queue_size=1)
        rospy.Service('~tts_offline', SetString, self.cbTTSOffline)
        rospy.Service('~toggle_hotword_detect', SetString,
                      self.cbToggleHotwordDetect)
        rospy.Service('~detect_command', GetCommandDetection,
                      self.cbDetectCommand)


        reader = threading.Thread(target=self.startHotwordDetect)
        # reader.daemon = True
        reader.start()

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbTTSOffline(self, params):
        data = params.data
        try:
            cmd = 'bash -c "cd %s && LD_LIBRARY_PATH=./  ./tts_offline %s && aplay ./tts_sample.wav"' % (
                self.bin_dir, data)
            print(cmd)
            os.system(cmd)
        except Exception as e:
            print(e)
        return SetStringResponse(data)

    def cbDetectCommand(self, params):
        print(params)
        data = params.data
        try:
            cmd = 'bash -c "cd %s && LD_LIBRARY_PATH=./  ./asr_offline %s "' % (
                self.bin_dir, data)
            if self.detector._running:
                self.detector.terminate()
            p = os.popen(cmd)
            res = p.read()
            result = re.findall(
                r"Result: \[ confidence=(\d+) grammar=(\d+) input=(.+) \]", res, re.S)
            print(cmd, res, result)
            if len(result) == 1 and len(result[0]) == 3:
                result = result[0]
                return GetCommandDetectionResponse(int(result[0]), int(result[1]), result[2])
        except Exception as e:
            print(e)
        return GetCommandDetectionResponse(0, 0, '')

    def cbToggleHotwordDetect(self, params):
        if params.data == 'open':
            if self.detector._running:
                return SetStringResponse('正在运行')
            else:
                self.hotwordDetect = True
                while not self.detector._running:
                    time.sleep(0.03)
                return SetStringResponse('已启动')

        elif params.data == 'close':
            if self.detector._running:
                # self.detector.terminate()
                self.hotwordDetect = False
                while self.detector._running:
                    time.sleep(0.03)
                return SetStringResponse('已停止')
            else:
                return SetStringResponse('没有运行')
        else:
            print(params.data)
            return SetStringResponse(params.data)

    def startHotwordDetect(self):
        while True:
            if self.hotwordDetect:
                print('starting')
                self.detector.start(
                    detected_callback=self.cbPublishHotwordDetect, interrupt_check=self.interrupt_check, sleep_time=0.03)
            else:
                if self.detector._running:
                    self.detector.terminate()
                    print('terminated')
            time.sleep(0.1)

    def interrupt_check(self):
        return self.hotwordDetect == False

    def cbPublishHotwordDetect(self, params=None):
        # self.detector.terminate()
        print('hotword detected')
        self.pubHotwordDetect.publish(String('hotword detected'))
        # self.startHotwordDetect()

    def onShutdown(self):
        self.hotwordDetect = False
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('smart_audio_node', anonymous=False)
    node = SmartAudioNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
