import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/d/Robotics_project/Robotics_Project/RPC_2/panda_kinematics/install/panda_kinematics'
