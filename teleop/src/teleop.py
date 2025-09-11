import os
import sys
import copy
from time import sleep, perf_counter
import pandas as pd
from ct_io.io_parser import IOParser
from ct_io.performance_metrics import PerformanceMetrics
from pose.pose import Pose
from pose.twist import Twist
from ctrl_interface.ctrl_interface import CtrlInterface
import ct_math.ct_math as ctm

def run_offline_mode(args):
        df = pd.read_csv(args.input_file)
        df = ctm.apply_coordinate_transformation(df)

        # key: rigid body name, value: Pose at current timestep
        source_curr_pose = {}
        source_prev_pose = {}

        target_pose = {}

        # key: rigid body name, value: twist at current timestep
        source_twist = {}
        target_twist = {}

        # Make the robot stand and wait a second to give it time
        CtrlInterface.stand(0, 0, 0)
        sleep(2)

        robot_orientation = CtrlInterface.get_robot_orientation()
        robot_position = CtrlInterface.get_robot_position()

        target_pose["Robot"] = Pose(0, robot_orientation[0], robot_orientation[1], 
                            robot_orientation[2], robot_orientation[3], 
                            robot_position[0], robot_position[1], robot_position[2])

        # At frame 0  (timesep 0)
        prev_timestep = df.at[(0, "Time (Seconds)")]
        source_prev_pose["LFoot"] = Pose(df.at[(0, "Time (Seconds)")], df.at[(0, "LFoot:Rotation:X")], df.at[(0, "LFoot:Rotation:Y")], df.at[(0, "LFoot:Rotation:Z")],
                                df.at[(0, "LFoot:Rotation:W")], df.at[(0, "LFoot:Position:X")], df.at[(0, "LFoot:Position:Y")], df.at[(0, "LFoot:Position:Z")])
        source_prev_pose["RFoot"] = Pose(df.at[(0, "Time (Seconds)")], df.at[(0, "RFoot:Rotation:X")], df.at[(0, "RFoot:Rotation:Y")], df.at[(0, "RFoot:Rotation:Z")],
                                df.at[(0, "RFoot:Rotation:W")], df.at[(0, "RFoot:Position:X")], df.at[(0, "RFoot:Position:Y")], df.at[(0, "RFoot:Position:Z")])
        source_prev_pose["Root"] = Pose(df.at[(0, "Time (Seconds)")], df.at[(0, "Waist:Rotation:X")], df.at[(0, "Waist:Rotation:Y")], df.at[(0, "Waist:Rotation:Z")],
                                df.at[(0, "Waist:Rotation:W")], df.at[(0, "Waist:Position:X")], df.at[(0, "Waist:Position:Y")], df.at[(0, "Waist:Position:Z")])
        
        performance_logger = PerformanceMetrics(source_prev_pose, target_pose, source_curr_pose,target_pose, source_twist, target_twist)

        for index, row in df.iloc[1:].iterrows(): # At frame 1 (timestep ~0.05)

            start_time = perf_counter()
            curr_timestep = row["Time (Seconds)"]

            # Update current pose with this frame's data
            source_curr_pose["LFoot"] = Pose(row["Time (Seconds)"], row["LFoot:Rotation:X"], row["LFoot:Rotation:Y"], row["LFoot:Rotation:Z"], row["LFoot:Rotation:W"], 
                                    row["LFoot:Position:X"], row["LFoot:Position:Y"], row["LFoot:Position:Z"])
            source_curr_pose["RFoot"] = Pose(row["Time (Seconds)"], row["RFoot:Rotation:X"], row["RFoot:Rotation:Y"], row["RFoot:Rotation:Z"], row["RFoot:Rotation:W"], 
                                    row["RFoot:Position:X"], row["RFoot:Position:Y"], row["RFoot:Position:Z"])
            source_curr_pose["Root"] = Pose(row["Time (Seconds)"], row["Waist:Rotation:X"], row["Waist:Rotation:Y"], row["Waist:Rotation:Z"], row["Waist:Rotation:W"], 
                                    row["Waist:Position:X"], row["Waist:Position:Y"], row["Waist:Position:Z"])
            
            
            # Calculate velocities
            dt = curr_timestep - prev_timestep
            source_twist["LFoot"] = Twist(source_curr_pose["LFoot"].timestep, ctm.linear_velocity(source_curr_pose["LFoot"], source_prev_pose["LFoot"], dt),
                                          ctm.angular_velocity(source_curr_pose["LFoot"], source_prev_pose["LFoot"], dt))
            source_twist["RFoot"] = Twist(source_curr_pose["RFoot"].timestep, ctm.linear_velocity(source_curr_pose["RFoot"], source_prev_pose["LFoot"], dt),
                                          ctm.angular_velocity(source_curr_pose["RFoot"], source_prev_pose["RFoot"], dt))
            source_twist["Root"] = Twist(source_curr_pose["Root"].timestep, ctm.linear_velocity(source_curr_pose["Root"], source_prev_pose["Root"], dt),
                                          ctm.angular_velocity(source_curr_pose["Root"], source_prev_pose["Root"], dt))

            robot_lv, robot_av = ctm.transform_cordinate_frame(source_twist["Root"].linear_velocity, source_twist["Root"].angular_velocity, robot_orientation)

            target_twist["Robot"] = Twist(curr_timestep, robot_lv, robot_av)

            performance_logger.log_metrics()
            performance_logger.print_metric_summary()
            
            # If you need to scale the values change this variable
            scale = 1.0
            CtrlInterface.walk(scale * target_twist["Robot"].linear_velocity[0], 
                               scale * target_twist["Robot"].linear_velocity[1], 
                               scale * target_twist["Robot"].angular_velocity[2])
            
            robot_orientation = CtrlInterface.get_robot_orientation()
            robot_position = CtrlInterface.get_robot_position()

            target_pose["Robot"] = Pose(curr_timestep, robot_orientation[0], robot_orientation[1], 
                                        robot_orientation[2], robot_orientation[3], 
                                        robot_position[0], robot_position[1], robot_position[2])

            # Sleep for the duration of the remaining time-step
            end_time = perf_counter()
            elapsed_time = end_time - start_time
            sleep(dt - elapsed_time)

            # Update timestep
            prev_timestep = curr_timestep

            # Update previous pose for next iteration
            source_prev_pose["LFoot"] = copy.deepcopy(source_curr_pose["LFoot"])
            source_prev_pose["RFoot"] = copy.deepcopy(source_curr_pose["RFoot"])
            source_prev_pose["Root"] = copy.deepcopy(source_curr_pose["Root"])

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
