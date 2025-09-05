import os
import sys
import copy
from time import sleep
import pandas as pd
from ct_io.io_parser import IOParser
from pose.pose import Pose
import ct_math.ct_math as ctm
import mpac_interface.mpac_interface as mpac

# Import the mpac_go2 python controller interface
mpac_go2_atnmy = "../mpac/mpac_go2/atnmy/"
print(os.path.exists(mpac_go2_atnmy))
sys.path.append(mpac_go2_atnmy)

import mpac_cmd

def run_offline_mode(args):
        df = pd.read_csv(args.input_file)

        # Make the robot stand and wait a second to give it time
        mpac_cmd.stand_idqp()
        sleep(2)

        # Start marching in place
        mpac_cmd.walk_idqp(vx= 0, vy = 0, vrz = 0)
        sleep(2) 

        # At frame 0  (timesep 0)
        waist_prev_pose = Pose(df.at[(0, "Time (Seconds)")], df.at[(0, "Waist:Rotation:X")], df.at[(0, "Waist:Rotation:Y")], df.at[(0, "Waist:Rotation:Z")],
                                df.at[(0, "Waist:Rotation:W")], df.at[(0, "Waist:Position:X")], df.at[(0, "Waist:Position:Y")], df.at[(0, "Waist:Position:Z")])
    

        for index, row in df.iloc[1:].iterrows(): # At frame 1 (timestep ~0.05)

            # Update current pose with this frame's data
            waist_curr_pose = Pose(row["Time (Seconds)"], row["Waist:Rotation:X"], row["Waist:Rotation:Y"], row["Waist:Rotation:Z"], row["Waist:Rotation:W"], 
                                row["Waist:Position:X"], row["Waist:Position:Y"], row["Waist:Position:Z"])
    
            # Calculate dt
            dt = waist_curr_pose.timestep - waist_prev_pose.timestep
            
            # Calculate velocities
            dt = waist_curr_pose.timestep - waist_prev_pose.timestep
            waist_lv = ctm.linear_velocity(waist_curr_pose, waist_prev_pose, dt)
            waist_av = ctm.angular_velocity(waist_curr_pose, waist_prev_pose, dt)
            
            robot_orientation = mpac.get_robot_orientation()
            robot_lv, robot_av = ctm.transform_cordinate_frame(waist_lv, waist_av, robot_orientation)
            
            print(f"\n------------------------CrossTele-----------------------")
            print(f"Timestep: {row["Time (Seconds)"]}")
            print(f"Human LV: {waist_lv}")
            print(f"Human AV: {waist_av}")

            print(f"Robot LV: {robot_lv}")
            print(f"Robot AV: {robot_av}")
            print("-----------------------------------------------------------")
            
            # If you need to scale the values change this variable
            scale = 1

            mpac_cmd.walk_idqp(vx=scale * robot_lv[0], vy=scale * robot_lv[1], vrz=scale * robot_av[2])

            # Sleep for the duration of the time-step
            sleep(dt)

            # Update previous pose for next iteration
            waist_prev_pose = copy.deepcopy(waist_curr_pose)

        mpac_cmd.hard_stop()