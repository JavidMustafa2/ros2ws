import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/javid/ros2ws/ros2ws/src/openrb_assignment/install/openrb_assignment'
