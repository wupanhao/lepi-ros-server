# 摄像头校准
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.031 image:=/usb_cam/image_raw camera:=/usb_cam
# 打开usb摄像头
roslaunch usb_cam usb_cam-test.launch

# roslib.js websocket api
rosrun rosbridge_server rosbridge_websocket

# 网页预览摄像头图像
rosrun web_video_server web_video_server

# apriltag 检测
roslaunch apriltag_ros continuous_detection.launch camera:=usb_cam

# 图像修正
ROS_NAMESPACE=usb_cam rosrun image_proc image_proc

# 打开摄像头
roslaunch usb_cam pi_cam-test_480x360.launch 

# duckietown camera source
# Software/catkin_ws/src/05-teleop/pi_camera

# export ROS_MASTER_URI="http://192.168.50.248:11311"
# export DISPLAY=192.168.50.162:0
# rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.031 image:=/decoder_low_freq/image/raw camera:=/camera

# 去除Windows行号,^M 用ctrl+V ctrl+M键入
# sed -i -e 's/^M/\n/g' camera_info_node.py