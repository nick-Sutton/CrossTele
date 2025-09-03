import os
import sys
from time import sleep
import pandas as pd
import argparse as arg
import copy
from io_parser import IOParser
from pose import Pose
import ct_math as m

# Import the mpac_go2 python controller interface
mpac_go2 = "../../../mpac_go2/atnmy/"
sys.path.append(mpac_go2 )
import mpac_cmd

def retarget_motion(obj_twist):
    return obj_twist

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
    print("-------------------------------------------------\n")

    # parse controller command config
    IOParser.parse_controller_config()

    if args.training:
        # Run in training mode
        pass
    elif args.input_mode == "offline" and args.io_mode == "mujoco":
        df = pd.read_csv(args.input_file)

        # At frame 0  (timesep 0)
        waist_prev_pose = Pose(df.at[(0, "Time (Seconds)")], df.at[(0, "Waist:Rotation:X")], df.at[(0, "Waist:Rotation:Y")], df.at[(0, "Waist:Rotation:Z")],
                                df.at[(0, "Waist:Rotation:W")], df.at[(0, "Waist:Position:X")], df.at[(0, "Waist:Position:Y")], df.at[(0, "Waist:Rotation:Z")])
        
        # At frame 1 (timestep ~0.05)
        waist_curr_pose = Pose(df.at[(1, "Time (Seconds)")], df.at[(1, "Waist:Rotation:X")], df.at[(1, "Waist:Rotation:Y")], df.at[(1, "Waist:Rotation:Z")],
                                df.at[(1, "Waist:Rotation:W")], df.at[(1, "Waist:Position:X")], df.at[(1, "Waist:Position:Y")], df.at[(1, "Waist:Rotation:Z")])

        mpac_cmd.stand_idqp()
        sleep(2)

        mpac_cmd.walk_idqp(0.25, 0, 0, 0)
        sleep(1)    
        for index, row in df.iloc[2:].iterrows():
            waist_twist = m.twist(waist_curr_pose, waist_prev_pose, waist_curr_pose.timestep - waist_prev_pose.timestep)

            # Retarget motion
            retargetted_motion = retarget_motion(waist_twist)

            # Call Mpac function
            # we need to perform a cordinate frame transformation
            print("\n------------------------CrossTele------------------------")
            print(retargetted_motion)
            print("-----------------------------------------------------------")
            mpac_cmd.walk_idqp(0.25, retargetted_motion[0, 3], retargetted_motion[1, 3], retargetted_motion[1, 0])    
            waist_prev_pose = copy.deepcopy(waist_curr_pose)

            waist_curr_pose.timestep = row["Time (Seconds)"]
            waist_curr_pose.orientationX = row["Waist:Rotation:X"]
            waist_curr_pose.orientationY= row["Waist:Rotation:Y"]
            waist_curr_pose.orientationZ= row["Waist:Rotation:Z"]
            waist_curr_pose.orientationW= row["Waist:Rotation:W"]

            waist_curr_pose.positionX = row["Waist:Position:X"]
            waist_curr_pose.positionY= row["Waist:Position:Y"]
            waist_curr_pose.positionZ= row["Waist:Position:Z"]

        mpac_cmd.hard_stop()

    elif args.input_mode == "offline" and args.io_mode == "hardware":
        IOParser.parse_offline_input(args.input_file)

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
