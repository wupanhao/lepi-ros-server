#!/usr/bin/python
#!coding:utf-8

import rospy

with open("/proc/cpuinfo", 'r') as f:
    lines = f.readlines()
    if "Pi 3 Model B" in lines[-1]:
        from pi3_driver_node import PiDriverNode
    else:
        from pi4_driver_node import PiDriverNode

if __name__ == '__main__':
    rospy.init_node('pi_driver_node', anonymous=False)
    node = PiDriverNode()
    # print(node.srvMotorsGetInfo(None))
    rospy.on_shutdown(node.onShutdown)
    # thread.start_new_thread(camera_node.startCaptureRawCV, ())
    rospy.spin()
