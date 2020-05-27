import rospy
import time
from pi_driver.srv import GetStrings,GetStringsRequest
from pi_driver.srv import SetString,SetStringRequest

available_nodes = [
	'/ubiquityrobot/camera_node',
	'/ubiquityrobot/apriltag_detector_node',
	'/ubiquityrobot/transfer_learning_node',
	'/ubiquityrobot/line_detector_node',
	'/ubiquityrobot/face_recognizer_node',
	'/ubiquityrobot/joystick_node',
		]

def test_get_alive_nodes():
	rospy.wait_for_service('/ubiquityrobot/pi_master_node/get_alive_nodes')
	get_alive_nodes = rospy.ServiceProxy('/ubiquityrobot/pi_master_node/get_alive_nodes', GetStrings)
	print(get_alive_nodes(GetStringsRequest()))
def test_shutdown_node(node_name = '/ubiquityrobot/camera_node'):
	rospy.wait_for_service('/ubiquityrobot/pi_master_node/shutdown_node')
	shutdown_node = rospy.ServiceProxy('/ubiquityrobot/pi_master_node/shutdown_node', SetString)
	print(shutdown_node(SetStringRequest(node_name)))
def test_launch_node( node_name = '/ubiquityrobot/camera_node' ):
	rospy.wait_for_service('/ubiquityrobot/pi_master_node/launch_node')
	launch_node = rospy.ServiceProxy('/ubiquityrobot/pi_master_node/launch_node', SetString)
	print(launch_node(SetStringRequest(node_name)))

if __name__ == '__main__':
	# test_shutdown_node()
	# time.sleep(5)
	for i in available_nodes:
		test_shutdown_node(i)
		# test_launch_node(i)