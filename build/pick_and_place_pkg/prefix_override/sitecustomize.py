import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bin1225/workspaces/ur_gz/install/pick_and_place_pkg'
