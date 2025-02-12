import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/inhwan/ros2_ws1/src/install/gazebo_tutorial'
