import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/henrique/projects/rasp_ros_iot/tello_ws/install/tello_sub'
