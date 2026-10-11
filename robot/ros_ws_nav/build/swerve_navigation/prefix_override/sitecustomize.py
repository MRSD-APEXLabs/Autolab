import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/labx/coding/Autolab/robot/ros_ws_nav/install/swerve_navigation'
