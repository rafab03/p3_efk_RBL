import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/AdR/src/p3_ws/p3_ekf_adr/install/p3_ekf_adr'
