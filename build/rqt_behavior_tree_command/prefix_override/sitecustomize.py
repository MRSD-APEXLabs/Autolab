import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ksriniv2/Autolab/install/rqt_behavior_tree_command'
