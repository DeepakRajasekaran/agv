import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lucifer/anscer_workspace/LineFollower/install/navigation_server'
