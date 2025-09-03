from NatNet.natnet_client import NatNetClient

def receive_new_frame(data_dict):
    order_list = ["frameNumber", "markerSetCount", "unlabeledMarkersCount", #type: ignore  # noqa F841
                  "rigidBodyCount", "skeletonCount", "labeledMarkerCount",
                  "timecode", "timecodeSub", "timestamp", "isRecording",
                  "trackedModelsChanged"]
    dump_args = False
    if dump_args is True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "= "
            if key in data_dict:
                out_string += data_dict[key] + " "
            out_string += "/"
        print(out_string)


def receive_new_frame_with_data(data_dict):
    order_list = ["frameNumber", "markerSetCount", "unlabeledMarkersCount", #type: ignore  # noqa F841
                  "rigidBodyCount", "skeletonCount", "labeledMarkerCount",
                  "timecode", "timecodeSub", "timestamp", "isRecording",
                  "trackedModelsChanged", "offset", "mocap_data"]
    dump_args = True
    if dump_args is True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "= "
            if key in data_dict:
                out_string += str(data_dict[key]) + " "
            out_string += "/"
        print(out_string)


# This is a callback function that gets connected to the NatNet client.
# It is called once per rigid body per frame.
def receive_rigid_body_frame(new_id, position, rotation):
    pass
    # print("Received frame for rigid body", new_id)
    # print("Received frame for rigid body", new_id," ",position," ",rotation)


def main():
    natnet_client = NatNetClient()

    natnet_client.new_frame_listener = receive_new_frame
    natnet_client.rigid_body_listener = receive_rigid_body_frame

    print("NatNet Python Client 4.3\n")
    is_running = natnet_client.run('d')

    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")
