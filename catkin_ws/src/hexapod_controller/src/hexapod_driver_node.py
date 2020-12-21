#!/usr/bin/python
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from hexapod_driver import SServo,Servo
from hexapod_controller.srv import GetOffsets,GetOffsetsResponse,SetOffsets,SetOffsetsResponse
import time
import yaml
import os
import threading

class HexapodDriverNode:
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.count = 0
        self.center = 1023/2
        self.speed = 1000
        self.cali_file = os.path.expanduser('~')+ '/Lepi_Data/ros/hexapod_controller/calibrations.yaml'
        self.pub_node_state = rospy.Publisher("/node_state", String, queue_size=1)
        self.pub_node_state.publish(String('{"node":"%s","state":"%s"}' % (self.node_name,"started") ))
        self.loadCaliFile()
        self.attatchServos()
        #self.toCenter()
        self.sub_joint_state = rospy.Subscriber("~joint_states", JointState, self.cbJointState, queue_size=1)
        self.sub_joint_angle = rospy.Subscriber("~joint_angles", JointState, self.cbJointAngle, queue_size=1)
        self.srv_get_offsets = rospy.Service("~get_offsets", GetOffsets, self.loadCaliFile)
        self.srv_set_offsets = rospy.Service("~set_offsets", SetOffsets, self.saveCaliFile)
        self.controller = threading.Thread(target=self.launchController)
        self.controller.start()
        rospy.loginfo("[%s] Initialized......" % (self.node_name))
        #os.system('bash -c "source /home/pi/nodejs.sh && node /home/pi/workspace/lepi-gui/app/hexapod/controller.js"')

    def launchController(self):
        os.system('bash -c "source /home/pi/nodejs.sh && node /home/pi/workspace/lepi-gui/app/hexapod/controller.js"')

    def attatchServos(self,port = '/dev/ttyUSB0'):
        try:
            self.servos = SServo(port)
        except Exception as e:
            self.servos = None
            print(e)
            exit()

    def toCenter(self):
        msg = JointState()
        # msg.position = self.offsets
        # self.cbJointAngle(msg)
        rads = [0 for i in range(18)]
        msg.position = rads
        self.cbJointState(msg)
    def cbServoAngles(self,angles):
        positions = [int(angles[i]/200.0*1023) for i in range(len(angles))]
        #for i in [2,5,8,11,14,17]:
        #    positions[i] = - positions[i]
        #servo_ids = [7, 8, 9, 10, 11, 12, 13, 14, 15, 4, 5, 6, 1, 2, 3, 16, 17, 18]
        servos = [Servo(i+1,positions[i]+self.center,speed=self.speed) for i in range(len(angles))]
        if self.servos is not None:
            self.servos.set_positions_sync(servos)

    def cbServoPositions(self,positions):
        #servo_ids = [7, 8, 9, 10, 11, 12, 13, 14, 15, 4, 5, 6, 1, 2, 3, 16, 17, 18]
        servos = [Servo(i+1,positions[i],speed=self.speed) for i in range(len(positions))]
        if self.servos is not None:
            self.servos.set_positions_sync(servos)

    def cbJointState(self,msg):
        positions = msg.position
        # print(msg)
        angles = [int(positions[i]/math.pi*180+self.offsets[i]) for i in range(len(positions))]
        self.cbServoAngles(angles)

    def cbJointAngle(self,msg):
        angles = [int(i) for i in msg.position]
        self.cbServoAngles(angles)

    def loadCaliFile(self,params=None):
        res = GetOffsetsResponse()
        try:
            with open(self.cali_file, 'r') as stream:
                data = yaml.load(stream)
                self.offsets = data['offsets']
        except Exception as e:
            print(e)
            self.offsets = [0 for i in range(18)]
        res.data.position = self.offsets
        return res

    def saveCaliFile(self,params=None):
        self.offsets = list(params.data.position)
        res = SetOffsetsResponse()
        with open(self.cali_file, 'w') as stream:
            yaml.dump({"offsets":self.offsets},stream)
        return res

    def onShutdown(self):
        self.pub_node_state.publish(String('{"node":"%s","state":"%s"}' % (self.node_name,"stopped") ))
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

if __name__ == '__main__':
    rospy.init_node('hexapod_driver_node', anonymous=False)
    node = HexapodDriverNode()
    rospy.on_shutdown(node.onShutdown)
    rospy.spin()
