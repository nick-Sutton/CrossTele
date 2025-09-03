import os
import pandas as pd

'''
    The data in the proccessed directory does to need to be reformated. It was already run through this script.
    If you need to transfer a CSV from the motive format to the training format you can either call the function
    in the scripts or change the raw data directory in the generate_reformatted_data() function
'''
def reformat_motive_csv(input_file, output_file):
    raw = pd.read_csv(input_file, header=None)

    # Detect header row
    header_row_idx = raw.index[raw.iloc[:, 0] == "Frame"][0]

    # Load data without motive header
    data = pd.read_csv(input_file, skiprows=header_row_idx)

    # Frame/Time
    frame_time = data.iloc[:, :2].copy()
    frame_time.columns = ["Frame", "Time (Seconds)"]

    # Rigid body names
    name_row_idx = raw.index[raw.iloc[:, 1] == "Name"][0]
    rigid_body_names = raw.iloc[name_row_idx, 2:].tolist()

    # Rotation/Position labels
    label_row_idx = raw.index[raw.iloc[:, 1] == "ID"][0] + 2
    labels_row = raw.iloc[label_row_idx, 2:].tolist()

    # Axis row
    axes_row = raw.iloc[header_row_idx, 2:].tolist()

    # Build column RigidBodyName:Rotation/Position:Axis
    col_names = []
    for rb_name, label, axis in zip(rigid_body_names, labels_row, axes_row):
        if pd.isna(rb_name):  # skip blanks
            continue
        col_names.append(f"{rb_name}:{label}:{axis}")

    # Assign columns
    rigid_body_data = data.iloc[:, 2:]
    rigid_body_data.columns = col_names

    rigid_body_data = rigid_body_data.drop(index=0).reset_index(drop=True)
    frame_time = frame_time.drop(index=0).reset_index(drop=True)

    # Reformatted DataFrame
    final_df = pd.concat([frame_time, rigid_body_data], axis=1)

    # Save
    final_df.to_csv(output_file, index=False)
    print(f"Reformatted CSV saved to {output_file}")


def generate_reformatted_data():

    # Dir's for reading and saving 
    raw_files_dir = "../dataset/TrackingDataV2/"
    processed_files_dir = "../dataset/FormattedDataV2/"

    # Create the output directory if it does not already exist
    os.makedirs(processed_files_dir, exist_ok=True)

    # Loop through input directory and run the reformatter on each file
    for file_name in os.listdir(raw_files_dir):
        raw_full_path = os.path.join(raw_files_dir, file_name)
        processed_full_path = os.path.join(processed_files_dir, file_name)
        if os.path.isfile(raw_full_path):
            reformat_motive_csv(raw_full_path, processed_full_path)

generate_reformatted_data()