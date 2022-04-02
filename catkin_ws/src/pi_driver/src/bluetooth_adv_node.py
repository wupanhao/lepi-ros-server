#!/usr/bin/python
import rospy
from pybleno import Bleno
from pyroute2 import IPRoute
from netifaces import interfaces, ifaddresses, AF_INET
import threading


def addr2hex(addr):
    nums = [int(i) for i in addr.split('.')]
    hex = '%02x%02x%02x%02x' % tuple(nums)
    return hex


def getIPs():
    ips = []
    for ifaceName in interfaces():
        ifs = ifaddresses(ifaceName)
        if AF_INET in ifs:
            addr = ifs[AF_INET][0]['addr']
            if ifaceName in ['wlan0', 'eth0']:
                ips.append(
                    {'iface': ifaceName, 'ip': addr, 'hex': addr2hex(addr)})
    return ips


def getAdvName():
    name = 'lepi'
    ips = getIPs()
    for ip in ips:
        name = name+'@'+ip['hex']
    return name


class BluetoothAdvNode:
    def __init__(self, name='lepi'):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" % (self.node_name))
        self.is_shutdown = False
        self.name = name
        self.bleno = Bleno()
        self.bleno.on('stateChange', self.onStateChange)

        self.bleno.on('advertisingStart', self.onAdvertisingStart)
        self.bleno.on('advertisingStop', self.onAdvertisingStop)
        self.loop = threading.Thread(target=self.start_listen_ip_loop)
        self.loop.daemon = True
        self.bleno.start()
        rospy.loginfo("[%s] Initialized......" % (self.node_name))

    def onStateChange(self, state):
        print('on -> stateChange: ' + state)

        if (state == 'poweredOn'):
            self.startAdvertising()
            self.loop.start()

        else:
            self.stopAdvertising()

    def startAdvertising(self):
        print('startAdvertising with name : ' + self.name)
        self.bleno.startAdvertising(self.name)

    def onAdvertisingStart(self, error):
        print('on -> advertisingStart: ' +
              ('error ' + error if error else 'success'))

    def onAdvertisingStop(self, error):
        print('on -> advertisingStaop: ' +
              ('error ' + error if error else 'success'))

    def stopAdvertising(self):
        self.bleno.stopAdvertising()

    def changeAdvertisingName(self, name):
        self.name = name
        self.stopAdvertising()
        self.startAdvertising()

    def onShutdown(self):
        self.is_shutdown = True
        self.bleno.disconnect()

    def start_listen_ip_loop(self):
        print('start_listen_ip_loop')
        while not self.is_shutdown:
            with IPRoute() as ipr:
                # With IPRoute objects you have to call bind() manually
                ipr.bind()
                for message in ipr.get():
                    if message['event'].find('ADDR') > 0 and message['family'] == 2:
                        print(message)
                        self.changeAdvertisingName(getAdvName())


if __name__ == '__main__':
    name = getAdvName()
    rospy.init_node('bluetooth_adv_node', anonymous=False)
    node = BluetoothAdvNode(name)
    # print(node.srvMotorsGetInfo(None))
    rospy.on_shutdown(node.onShutdown)
    # thread.start_new_thread(camera_node.startCaptureRawCV, ())
    rospy.spin()
    # node.start_listen_ip_loop()
