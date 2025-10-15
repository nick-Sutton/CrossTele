import os
import sys
import copy
from natnet import DataDescriptions, DataFrame, NatNetClient
from time import sleep, perf_counter
import pandas as pd
from ct_io.io_parser import IOParser
from ct_io.performance_metrics import PerformanceMetrics
from pose.human import Human
from pose.pose import Pose
from pose.twist import Twist
from ctrl_interface.ctrl_interface import CtrlInterface
import ct_math.ct_math as ctm

def run_offline_mode(args):
        df = pd.read_csv(args.input_file)
        #df = ctm.apply_coordinate_transformation(df)

        # Define dataset Sampling frequency and construct human object
        sampling_freq = 240.0
        human = Human(sampling_freq)

        # Make the robot stand and wait a second to give it time
        CtrlInterface.stand(0, 0, 0)
        sleep(2)

        robot_orientation = CtrlInterface.get_robot_orientation()
        robot_position = CtrlInterface.get_robot_position()

        target_pose = {}
        target_twist = {}

        target_pose["Robot"] = Pose(0, robot_orientation[0], robot_orientation[1], 
                            robot_orientation[2], robot_orientation[3], 
                            robot_position[0], robot_position[1], robot_position[2])

        # At frame 0  (timesep 0)
        prev_timestep = df.at[(0, "Time (Seconds)")]

        human.prev_pose["LFoot"] = Pose(df.at[(0, "Time (Seconds)")], df.at[(0, "LFoot:Rotation:X")], df.at[(0, "LFoot:Rotation:Y")], df.at[(0, "LFoot:Rotation:Z")],
                                df.at[(0, "LFoot:Rotation:W")], df.at[(0, "LFoot:Position:X")], df.at[(0, "LFoot:Position:Y")], df.at[(0, "LFoot:Position:Z")])
        human.prev_pose["RFoot"] = Pose(df.at[(0, "Time (Seconds)")], df.at[(0, "RFoot:Rotation:X")], df.at[(0, "RFoot:Rotation:Y")], df.at[(0, "RFoot:Rotation:Z")],
                                df.at[(0, "RFoot:Rotation:W")], df.at[(0, "RFoot:Position:X")], df.at[(0, "RFoot:Position:Y")], df.at[(0, "RFoot:Position:Z")]) 
        human.prev_pose["Root"] = Pose(df.at[(0, "Time (Seconds)")], df.at[(0, "Waist:Rotation:X")], df.at[(0, "Waist:Rotation:Y")], df.at[(0, "Waist:Rotation:Z")],
                                df.at[(0, "Waist:Rotation:W")], df.at[(0, "Waist:Position:X")], df.at[(0, "Waist:Position:Y")], df.at[(0, "Waist:Position:Z")])

        # Set up Performance Logger
        performance_logger = PerformanceMetrics(human.prev_pose, target_pose, human, target_pose, human.curr_twist, target_twist)

        for index, row in df.iloc[1:].iterrows(): # At frame 1 til last timestep
            start_time = perf_counter()
            curr_timestep = row["Time (Seconds)"]

            # Update current pose with this frame's data
            human.curr_pose["LFoot"] = Pose(row["Time (Seconds)"], row["LFoot:Rotation:X"], row["LFoot:Rotation:Y"], row["LFoot:Rotation:Z"], row["LFoot:Rotation:W"], 
                                    row["LFoot:Position:X"], row["LFoot:Position:Y"], row["LFoot:Position:Z"])
            human.curr_pose["RFoot"] = Pose(row["Time (Seconds)"], row["RFoot:Rotation:X"], row["RFoot:Rotation:Y"], row["RFoot:Rotation:Z"], row["RFoot:Rotation:W"], 
                                    row["RFoot:Position:X"], row["RFoot:Position:Y"], row["RFoot:Position:Z"])
            human.curr_pose["Root"] = Pose(row["Time (Seconds)"], row["Waist:Rotation:X"], row["Waist:Rotation:Y"], row["Waist:Rotation:Z"], row["Waist:Rotation:W"], 
                                    row["Waist:Position:X"], row["Waist:Position:Y"], row["Waist:Position:Z"])
            
            
            # Calculate velocities
            dt = curr_timestep - prev_timestep
            human.curr_twist["LFoot"] = Twist(human.curr_pose["LFoot"].timestep, ctm.linear_velocity(human.curr_pose["LFoot"], human.prev_pose["LFoot"], dt),
                                          ctm.angular_velocity(human.curr_pose["LFoot"], human.prev_pose["LFoot"], dt))
            human.curr_twist["RFoot"] = Twist(human.curr_pose["RFoot"].timestep, ctm.linear_velocity(human.curr_pose["RFoot"], human.prev_pose["RFoot"], dt),
                                          ctm.angular_velocity(human.curr_pose["RFoot"], human.prev_pose["RFoot"], dt))
            human.curr_twist["Root"] = Twist(human.curr_pose["Root"].timestep, ctm.linear_velocity(human.curr_pose["Root"], human.prev_pose["Root"], dt),
                                          ctm.angular_velocity(human.curr_pose["Root"], human.prev_pose["Root"], dt))
            
            # Transformations and feature extraction
            robot_lv, robot_av = ctm.transform_cordinate_frame(human.curr_twist["Root"].linear_velocity, human.curr_twist["Root"].angular_velocity, robot_orientation)

            target_twist["Robot"] = Twist(curr_timestep, robot_lv, robot_av)

            features = {}
            human.extract_gait_features(features)

            performance_logger.log_metrics()
            #performance_logger.print_metric_summary()

            # If you need to scale the values change this variable
            scale = 1.0
            CtrlInterface.walk(scale * target_twist["Robot"].linear_velocity[0], 
                               scale * target_twist["Robot"].linear_velocity[1], 
                               0.0 * target_twist["Robot"].angular_velocity[2])
            
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

            human.prev_pose = human.curr_pose.copy()

        CtrlInterface.hard_stop()

        print("Generating Logs...")
        performance_logger.plot_metrics()
        print("Done")



# Feed Forward NN
# Recurrent Neural Network
# LSTM encoder -> LSTM decoder
# Just use a Transformer bro
# GRU
# TCN
# TGNN
num_frames = 0
def receive_new_frame(data_frame: DataFrame):
    global num_frames
    num_frames += 1
    print(num_frames)

def main():
    # python3 teleop/src/teleop.py --input_mode=offline --io_mode=mujoco --input_file=./training/dataset/FormattedDataV2/Walk_backwards_000.csv
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
        #natnet_config = IOParser.parse_natnet_config()

        # Create Client
        streaming_client = NatNetClient(server_ip_address='169.254.128.60', 
                                        local_ip_address='10.154.55.236', 
                                        use_multicast=False)
        streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)

        

        streaming_client.connect(7)
        print(streaming_client.connected)


        for i in range(50):
            streaming_client.update_sync()


        # Set up Natnet client
        pass
    elif args.input_mode == "online" and args.io_mode == "hardware":
        pass
    else:
        parser.error("Could not recognize commands")

if __name__ == "__main__":
    main()
