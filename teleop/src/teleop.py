import os
import sys
import copy
from time import sleep
import pandas as pd
from ct_io.io_parser import IOParser
from pose.pose import Pose
from ctrl_interface.ctrl_interface import CtrlInterface
import ct_math.ct_math as ctm

def run_offline_mode(args):
        df = pd.read_csv(args.input_file)

        # Make the robot stand and wait a second to give it time
        CtrlInterface.stand(0, 0, 0)
        sleep(2)

        # Start marching in place
        CtrlInterface.walk(0, 0, 0)
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
            
            robot_orientation = CtrlInterface.get_robot_orientation()
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
            CtrlInterface.walk(scale * robot_lv[0], scale * robot_lv[1], scale * robot_av[2])

            # Sleep for the duration of the time-step
            sleep(dt)

            # Update previous pose for next iteration
            waist_prev_pose = copy.deepcopy(waist_curr_pose)

        CtrlInterface.hard_stop()

def main():
    # Create arg parser and get cmd arguments
    parser = IOParser.get_cmd_parser()
    args = parser.parse_args()

    # ------------------------------------ Arg Error handling ----------------------------------- #
    # Training mode must be the only argument
    if args.training:
        if args.input_mode or args.io_mode or args.input_file:
            parser.error("--training cannot be used with --input_mode, --io_mode, or --input_file")
    
    # Offline mode requires input_file
    if args.input_mode == 'offline' and not args.input_file:
        parser.error("--input_mode offline requires --input_file")
    
    # Input file should be a valid CSV file for offline mode
    if args.input_file:
        if args.input_mode != 'offline':
            parser.error("--input_file can only be used with --input_mode offline")
        if not args.input_file.endswith('.csv'):
            parser.error("Input file must be a CSV file")
        if not os.path.exists(args.input_file):
            parser.error(f"Input file not found: {args.input_file}")
    
    # io_mode requires input_mode
    if (args.io_mode and not args.input_mode) or args.input_mode and not args.io_mode:
        parser.error("--input_mode and --io_mode required")
    
    # Online mode shouldn't have input_file
    if args.input_mode == 'online' and args.input_file:
        parser.error("--input_file cannot be used with --input_mode online")
    
    # ------------------------------------ Arg Error handling ----------------------------------- #

    print("\n--------------------CrossTele--------------------")
    print(f"Running with input_mode={args.input_mode}, io_mode={args.io_mode}")

    # parse controller command config
    IOParser.parse_controller_config()

    if args.training:
        # Run in training mode
        pass
    elif args.input_mode == "offline" and args.io_mode == "mujoco":
        # Call Offline mode loop
        run_offline_mode(args)
    elif args.input_mode == "offline" and args.io_mode == "hardware":
        pass
    elif args.input_mode == "online" and args.io_mode == "mujoco":
        # parse natnet_Config
        IOParser.parse_natnet_config()

        # Set up Natnet client
        pass
    elif args.input_mode == "online" and args.io_mode == "hardware":
        pass
    else:
        parser.error("Could not recognize commands")

if __name__ == "__main__":
    main()
