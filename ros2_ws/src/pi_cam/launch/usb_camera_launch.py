from launch import LaunchDescription
from launch_ros.actions import Node
ns = 'ubiquityrobot'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pi_cam',
            namespace=ns,
            executable='usb_camera',
            # name='usb_camera'
        )
    ])
