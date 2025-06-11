import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/strohmo/autonomous-system/install/tensorrt_cone_detect'
