import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ashmu/ros2_ws/RS2/install/svg_waypoint_publisher'
