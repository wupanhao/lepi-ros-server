#!/usr/bin/python
# from pi_driver import Lepi
from pi_driver.msg import ButtonEvent

import rospkg
import rospy

class ButtonDriverNode:
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing......" % (self.node_name))
		self.pub_button_event =rospy.Publisher("~button_event", ButtonEvent, queue_size=1)
		self.rate = 1
		self.r = rospy.Rate(self.rate)
		self.timer_init = rospy.Timer(rospy.Duration.from_sec(1.0/self.rate), self.pubTest)
		rospy.loginfo("[%s] Initialized......" % (self.node_name))

	def pubTest(self,event):
		# print(msg)
		e = ButtonEvent()
		e.type = 1
		e.value = 39
		self.pub_button_event.publish(e)


if __name__ == '__main__':
	rospy.init_node('button_driver_node', anonymous=False)
	node = ButtonDriverNode()
	# rospy.on_shutdown(node.onShutdown)
	# thread.start_new_thread(camera_node.startCaptureRawCV, ())
	rospy.spin()
