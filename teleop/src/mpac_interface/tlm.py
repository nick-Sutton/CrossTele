import os
import sys
import ctypes

# Import the mpac_go2 python tlm interface
mpac_go2_atnmy = "../mpac/mpac_go2/tlm/"
print(os.path.exists(mpac_go2_atnmy))
sys.path.append(mpac_go2_atnmy)