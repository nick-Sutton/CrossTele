import os
import pandas as pd

from teleop.src.ct_math.ct_math import angular_velocity, linear_velocity
from teleop.src.pose.human import Human
from teleop.src.pose.pose import Pose
from teleop.src.pose.twist import Twist

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
    raw_files_dir = "../training/dataset/TrackingDataV4/"
    processed_files_dir = "../training/dataset/FormattedDataV2/"

    # Create the output directory if it does not already exist
    os.makedirs(processed_files_dir, exist_ok=True)

    # Loop through input directory and run the reformatter on each file
    for file_name in os.listdir(raw_files_dir):
        raw_full_path = os.path.join(raw_files_dir, file_name)
        processed_full_path = os.path.join(processed_files_dir, file_name)
        if os.path.isfile(raw_full_path):
            reformat_motive_csv(raw_full_path, processed_full_path)


# Human Gaits 
# Stand
# Quasi steady
# trot 
# running
def lable_data(input_file, output_file):
    gait_keywords = {
        'quasi': 'quasi',
        'walk': 'walk',
        'jog': 'jog',
        'face': 'stand'
    }

    df = pd.read_csv(input_file)

    filename = os.path.basename(input_file)
    file_class = filename.split('_')[0].lower()

    # Map to standard gait label
    gait_type = gait_keywords.get(file_class, file_class)
    
    # Add label to all rows
    df['gait_type'] = gait_type

    # Save
    df.to_csv(output_file, index=False)
    print(f"Labled CSV saved to {output_file}")
    

def generate_lable_data():

    # Dir's for reading and saving 
    raw_files_dir = "training/dataset/UnlabledData/"
    processed_files_dir = "training/dataset/LabledData/"

    # Create the output directory if it does not already exist
    os.makedirs(processed_files_dir, exist_ok=True)

    for file_name in os.listdir(raw_files_dir):
        raw_full_path = os.path.join(raw_files_dir, file_name)
        processed_full_path = os.path.join(processed_files_dir, file_name)
        if os.path.isfile(raw_full_path):
            lable_data(raw_full_path, processed_full_path)

def extract_gait_label_from_filename(filename):
    """
    Extract gait label from filename.
    Example: 'Walk_forward_001.csv' -> 'walk'
    """
    gait_keywords = {
        'quasi': 'quasi',
        'walk': 'walk',
        'jog': 'jog',
        'face': 'stand'
    }
    
    first_word = os.path.basename(filename).split('_')[0].lower()
    return gait_keywords.get(first_word, first_word)

def get_features(input_file, output_file):
    df = pd.read_csv(input_file)
    
    # Extract gait label from filename
    gait_label = extract_gait_label_from_filename(input_file)
    
    print(f"Processing: {os.path.basename(input_file)}")
    print(f"  Gait: {gait_label}")
    print(f"  Frames: {len(df)}")
    
    # Define sampling frequency (matches your real-time setup)
    sampling_freq = 240.0
    human = Human(sampling_freq)
    
    # Initialize with frame 0
    prev_timestep = df.at[0, "Time (Seconds)"]
    
    human.prev_pose["LFoot"] = Pose(
        df.at[0, "Time (Seconds)"], 
        df.at[0, "LFoot:Rotation:X"], df.at[0, "LFoot:Rotation:Y"], 
        df.at[0, "LFoot:Rotation:Z"], df.at[0, "LFoot:Rotation:W"],
        df.at[0, "LFoot:Position:X"], df.at[0, "LFoot:Position:Y"], 
        df.at[0, "LFoot:Position:Z"]
    )
    
    human.prev_pose["RFoot"] = Pose(
        df.at[0, "Time (Seconds)"], 
        df.at[0, "RFoot:Rotation:X"], df.at[0, "RFoot:Rotation:Y"], 
        df.at[0, "RFoot:Rotation:Z"], df.at[0, "RFoot:Rotation:W"],
        df.at[0, "RFoot:Position:X"], df.at[0, "RFoot:Position:Y"], 
        df.at[0, "RFoot:Position:Z"]
    )
    
    human.prev_pose["Root"] = Pose(
        df.at[0, "Time (Seconds)"], 
        df.at[0, "Waist:Rotation:X"], df.at[0, "Waist:Rotation:Y"], 
        df.at[0, "Waist:Rotation:Z"], df.at[0, "Waist:Rotation:W"],
        df.at[0, "Waist:Position:X"], df.at[0, "Waist:Position:Y"], 
        df.at[0, "Waist:Position:Z"]
    )
    
    # Store all extracted features
    all_features = []
    
    # Process each frame (starting from frame 1)
    for index, row in df.iloc[1:].iterrows():
        curr_timestep = row["Time (Seconds)"]
        
        # Update current pose with this frame's data
        human.curr_pose["LFoot"] = Pose(
            row["Time (Seconds)"], 
            row["LFoot:Rotation:X"], row["LFoot:Rotation:Y"], 
            row["LFoot:Rotation:Z"], row["LFoot:Rotation:W"],
            row["LFoot:Position:X"], row["LFoot:Position:Y"], 
            row["LFoot:Position:Z"]
        )
        
        human.curr_pose["RFoot"] = Pose(
            row["Time (Seconds)"], 
            row["RFoot:Rotation:X"], row["RFoot:Rotation:Y"], 
            row["RFoot:Rotation:Z"], row["RFoot:Rotation:W"],
            row["RFoot:Position:X"], row["RFoot:Position:Y"], 
            row["RFoot:Position:Z"]
        )
        
        human.curr_pose["Root"] = Pose(
            row["Time (Seconds)"], 
            row["Waist:Rotation:X"], row["Waist:Rotation:Y"], 
            row["Waist:Rotation:Z"], row["Waist:Rotation:W"],
            row["Waist:Position:X"], row["Waist:Position:Y"], 
            row["Waist:Position:Z"]
        )
        
        # Calculate velocities
        dt = curr_timestep - prev_timestep
        
        human.curr_twist["LFoot"] = Twist(
            human.curr_pose["LFoot"].timestep,
            linear_velocity(human.curr_pose["LFoot"], human.prev_pose["LFoot"], dt),
            angular_velocity(human.curr_pose["LFoot"], human.prev_pose["LFoot"], dt)
        )
        
        human.curr_twist["RFoot"] = Twist(
            human.curr_pose["RFoot"].timestep,
            linear_velocity(human.curr_pose["RFoot"], human.prev_pose["RFoot"], dt),
            angular_velocity(human.curr_pose["RFoot"], human.prev_pose["RFoot"], dt)
        )
        
        human.curr_twist["Root"] = Twist(
            human.curr_pose["Root"].timestep,
            linear_velocity(human.curr_pose["Root"], human.prev_pose["Root"], dt),
            angular_velocity(human.curr_pose["Root"], human.prev_pose["Root"], dt)
        )
        
        # Extract features (same as real-time)
        features = {}
        human.extract_gait_features(features)
        
        # Add gait label
        features['gait_type'] = gait_label
        
        # Add frame info
        features['frame_id'] = index
        features['timestamp'] = curr_timestep
        
        all_features.append(features)
        
        # Update for next iteration
        prev_timestep = curr_timestep
        human.prev_pose = human.curr_pose.copy()
    
    # Convert to DataFrame and save
    features_df = pd.DataFrame(all_features)
    
    # Reorder columns: frame_id, timestamp, gait_type, then all features
    cols = ['frame_id', 'timestamp', 'gait_type'] + [col for col in features_df.columns 
                                                       if col not in ['frame_id', 'timestamp', 'gait_type']]
    features_df = features_df[cols]
    
    features_df.to_csv(output_file, index=False)
    print(f"  Saved {len(features_df)} frames to {os.path.basename(output_file)}")
    
    return features_df

def generate_feature_data():
    """
    Process all raw mocap CSV files and extract features.
    Matches your directory structure.
    """
    # Directories for reading and saving
    raw_files_dir = "training/dataset/LabeledData/"
    processed_files_dir = "training/dataset/TrainingData/"
    
    # Create the output directory if it does not already exist
    os.makedirs(processed_files_dir, exist_ok=True)
    
    print("="*60)
    print("Feature Extraction from Raw Mocap Data")
    print("="*60)
    print(f"Input directory: {raw_files_dir}")
    print(f"Output directory: {processed_files_dir}")
    print()
    
    # Get all CSV files
    csv_files = [f for f in os.listdir(raw_files_dir) if f.endswith('.csv')]
    
    if len(csv_files) == 0:
        print(f"No CSV files found in {raw_files_dir}")
        return
    
    print(f"Found {len(csv_files)} CSV files to process\n")
    
    all_processed_dfs = []
    
    # Process each file
    for file_name in csv_files:
        raw_full_path = os.path.join(raw_files_dir, file_name)
        processed_full_path = os.path.join(processed_files_dir, file_name)
        
        if os.path.isfile(raw_full_path):
            try:
                df = get_features(raw_full_path, processed_full_path)
                all_processed_dfs.append(df)
            except Exception as e:
                print(f"  ERROR: {e}")
        print()
    
    # Combine all processed files into one dataset
    if all_processed_dfs:
        combined_df = pd.concat(all_processed_dfs, ignore_index=True)
        combined_path = os.path.join(processed_files_dir, "combined_features.csv")
        combined_df.to_csv(combined_path, index=False)
        
        print("="*60)
        print("Processing Complete!")
        print("="*60)
        print(f"Total frames processed: {len(combined_df)}")
        print(f"\nGait distribution:")
        print(combined_df['gait_type'].value_counts())
        print(f"\nCombined dataset saved to: {combined_path}")
        print("="*60)
    else:
        print("No files were successfully processed.")