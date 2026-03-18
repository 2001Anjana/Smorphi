import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anjana/Documents/PR2/Simulation_03_17/install/smorphi_2d_simulation'
