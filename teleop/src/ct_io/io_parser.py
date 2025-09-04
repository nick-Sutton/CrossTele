import csv
import os
import numpy as np
import argparse as arg

class IOParser():
    def __init__(self):
        pass

    def get_cmd_parser():
        cmd_parser= arg.ArgumentParser("!Fill in Help Message Later!")
        input_mode_group = cmd_parser.add_mutually_exclusive_group(required=True)

        input_mode_group.add_argument('--input_mode', choices=['offline', 'online', 'training'], help="Run the teleop controller in with dataset input or live streamed data input")
        input_mode_group.add_argument('--training', action='store_true', help=" Run in training mode")

        cmd_parser.add_argument('--io_mode', choices=['mujoco', 'hardware'], help="Run the teleop controller in simulation or on hardware")
        cmd_parser.add_argument('--input_file', type=str, help="Input CSV file (required for offline mode)")

        return cmd_parser

    def parse_controller_config():
        pass

    def parse_natnet_config():
        pass
