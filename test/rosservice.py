#! /usr/bin/env python

import rospy
from pi_driver.srv import GetInt32, GetInt32Request
from pi_driver.msg import U8Int32
import time
# init a node as usual
rospy.init_node('ros_service_client')
topic_name = '/ubiquityrobot/pi_driver_node/motor_set_speed'
topic = rospy.Publisher(topic_name, U8Int32, queue_size=1)
count = 1000


def test_service():
    # wait for this sevice to be running
    rospy.wait_for_service('/ubiquityrobot/pi_driver_node/sensor_get_value')

    # Create the connection to the service. Remember it's a Trigger service
    service = rospy.ServiceProxy(
        '/ubiquityrobot/pi_driver_node/sensor_get_value', GetInt32)

    start = time.time()
    param = GetInt32Request(1)

    for i in range(count):
        # Create an object of the type TriggerRequest. We nned a TriggerRequest for a Trigger service
        # Now send the request through the connection
        result = service(param)
        # Done
        # print(result)

    end = time.time()
    print('time cost %f ms after %d times call ros service' %
          ((end-start)*1000, count))


def test_get_param():
    start = time.time()
    for i in range(count):
        result = rospy.get_param("/variable/a", 0.2)
    print(result)
    end = time.time()
    print('time cost %f ms after %d times call ros param get' %
          ((end-start)*1000, count))


def test_set_param():
    start = time.time()
    for i in range(count):
        result = rospy.set_param("/variable/a", i)
    print(result)
    end = time.time()
    print('time cost %f ms after %d times call ros param set' %
          ((end-start)*1000, count))


def test_set_publish():
    start = time.time()
    for i in range(count):
        msg = U8Int32(1, i % 100)
        # time.sleep(0.0001)
        topic.publish(msg)
    print(msg)
    end = time.time()
    print('time cost %f ms after %d times call ros topic publish' %
          ((end-start)*1000, count))


if __name__ == '__main__':
    test_set_publish()
    # time.sleep(2)
    test_service()
    test_get_param()
    test_set_param()
