#!/usr/bin/python3
#!coding:utf-8
import os
import rospy
import paddlehub as hub
from pathlib import Path
from pi_driver.srv import GetString, GetStringResponse

root_dir = os.path.expanduser(
    '~')+'/Lepi_Data/ros/smart_audio_node'
tts_dir = os.path.expanduser(
    '~')+'/Lepi_Data/Music'


class SmartAudioNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        # self.asr_model = hub.Module(
        #    name='u2_conformer_aishell',
        #    version='1.0.0')
        self.tts_model = hub.Module(
            name='fastspeech2_baker',
            version='1.0.0')
        self.tts_model.output_dir = Path(root_dir)
        rospy.Service('~tts_offline', GetString, self.cbTTSOffline)
        rospy.Service('~detect_command', GetString,
                      self.cbDetectCommand)
        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def cbTTSOffline(self, params):
        data = params.data
        try:
            self.tts_model.generate([data])
            cmd = 'aplay %s' % os.path.join(root_dir, '1.wav')
            os.system(cmd)
            cmd = 'cp %s %s' % (os.path.join(root_dir, '1.wav'),
                                os.path.join(tts_dir, data+'.wav'))
            os.system(cmd)
        except Exception as e:
            print(e)
        return GetStringResponse(data)

    def cbDetectCommand(self, params):
        print(params)
        try:
            length = int(params.data)
            if length > 0 and length < 20:
                pass
            else:
                length = 5
            file = os.path.join(root_dir, 'record.wav')
            os.system(
                'arecord -r 16000 -f S16_LE  -d %d %s' % (length, file))
            text = self.asr_model.speech_recognize(file)
            print(text)
            return GetStringResponse(text)
        except Exception as e:
            print(e)
        return GetStringResponse('')

    def onShutdown(self):
        self.hotwordDetect = False
        rospy.loginfo("[%s] Shutdown." % (self.node_name))


if __name__ == '__main__':
    rospy.init_node('smart_audio_node', anonymous=False)
    node = SmartAudioNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
