import os
import sys
import ct_math.ct_math as ctm

# Import the mpac_go2 python controller interface
mpac_go2_atnmy = "../mpac/mpac_go2/atnmy/"
print(os.path.exists(mpac_go2_atnmy))
sys.path.append(mpac_go2_atnmy)

import mpac_cmd

class CtrlInterface():
    def __init__(self):
        pass

    def walk(vx, vy, vrz):
        mpac_cmd.walk_idqp(h=0.25,vx=vx, vy=vy, vrz=vrz)

    def stand(rx, ry, rz):
        mpac_cmd.stand_idqp(h=0.25, rx=rx, ry=ry, rz=rz)

    def bound(vx):
        mpac_cmd.bound(vx=vx)

    def jump(vx, vy, vz):
        mpac_cmd.jump(x_vel=vx, y_vel=vy, z_vel=vz)

    def land():
        mpac_cmd.land()

    def soft_stop():
        mpac_cmd.soft_stop()

    def hard_stop():
        mpac_cmd.hard_stop()

    def get_tlm_data():
        return mpac_cmd.get_tlm_data()

    def get_robot_orientation():
        # Get the robots current orientation
        tlm_data = mpac_cmd.get_tlm_data()
        if isinstance(tlm_data, list):
            r_roll, r_pitch, r_yaw = tlm_data[0]["q"][3:6]  # First robot
        else:
            r_roll, r_pitch, r_yaw = tlm_data["q"][3:6]     # Single robot

        # Convert eular to quat
        robot_orientation = ctm.eular_to_quat(r_roll, r_pitch, r_yaw)

        return robot_orientation