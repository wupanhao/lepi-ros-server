#!/usr/bin/python
#!coding:utf-8
import os
import rospy
from pi_driver.srv import SetString, SetStringResponse


class SmartAudioNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.bin_dir = os.path.expanduser(
            '~')+'/Lepi_Data/ros/smart_audio_node'
        # self.pub_detections = rospy.Publisher("~image_text", CompressedImage, queue_size=1)

        rospy.Service('~tts_offline', SetString, self.cbTTSOffline)

        # self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImg ,  queue_size=1)
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

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('smart_audio_node', anonymous=False)
    node = SmartAudioNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
