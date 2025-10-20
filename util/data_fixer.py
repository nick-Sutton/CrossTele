import pandas as pd
import os
import numpy as np

from teleop.src.ct_math.ct_math import angular_velocity, linear_velocity
from teleop.src.pose.human import Human
from teleop.src.pose.pose import Pose
from teleop.src.pose.twist import Twist

# Assuming you have these classes defined elsewhere
# from your_module import Human, Pose, Twist, linear_velocity, angular_velocity

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


def classify_gait_by_velocity(root_linear_vel, root_angular_vel, filename_gait, velocity_threshold=0.1):
    """
    Classify gait as 'stand' if velocities are low, otherwise use filename gait type.
    
    Parameters:
    - root_linear_vel: numpy array of [x, y, z] linear velocity
    - root_angular_vel: numpy array of [x, y, z] angular velocity
    - filename_gait: gait type extracted from filename
    - velocity_threshold: threshold below which motion is considered standing
    
    Returns:
    - 'stand' if motion is minimal, otherwise filename_gait
    """
    # Calculate magnitude of linear velocity (in XZ plane, ignoring Y which is vertical)
    linear_mag = np.sqrt(root_linear_vel[0]**2 + root_linear_vel[1]**2)
    
    # Calculate magnitude of angular velocity
    #angular_mag = np.sqrt(np.sum(root_angular_vel**2))
    
    # If both velocities are below threshold, classify as standing
    if linear_mag < velocity_threshold and linear_mag > -velocity_threshold:
        return 'stand'
    else:
        return filename_gait


def get_features(input_file, output_file, velocity_threshold=0.05):
    """
    Extract features from raw mocap CSV without requiring pre-labeled data.
    Classifies gait based on velocity values and filename.
    """
    df = pd.read_csv(input_file)
    
    # Extract gait label from filename
    filename_gait = extract_gait_label_from_filename(input_file)
    
    print(f"Processing: {os.path.basename(input_file)}")
    print(f"  Filename Gait: {filename_gait}")
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
        human.extract_gait_features(features, row['Frame'])
        
        # Classify gait based on velocity
        root_linear_vel = np.array([
            features['root_linear_velocity_x'],
            features['root_linear_velocity_y'],
            features['root_linear_velocity_z']
        ])
        root_angular_vel = np.array([
            features['root_angular_velocity_x'],
            features['root_angular_velocity_y'],
            features['root_angular_velocity_z']
        ])
        
        gait_type = classify_gait_by_velocity(
            root_linear_vel, 
            root_angular_vel, 
            filename_gait,
            velocity_threshold
        )
        
        # Add gait label
        features['gait_type'] = gait_type
        
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
    
    # Print statistics
    gait_counts = features_df['gait_type'].value_counts()
    print(f"  Gait distribution in file:")
    for gait, count in gait_counts.items():
        print(f"    {gait}: {count} frames ({count/len(features_df)*100:.1f}%)")
    print(f"  Saved {len(features_df)} frames to {os.path.basename(output_file)}")
    
    return features_df


def generate_feature_data(velocity_threshold=0.05):
    """
    Process all raw mocap CSV files and extract features.
    No pre-labeled data required - classifies based on velocity and filename.
    
    Parameters:
    - velocity_threshold: threshold for classifying standing vs moving (default 0.05)
    """
    # Directories for reading and saving
    raw_files_dir = "training/dataset/UnlabeledData/"
    processed_files_dir = "training/dataset/TrainingDataV2/"
    
    # Create the output directory if it does not already exist
    os.makedirs(processed_files_dir, exist_ok=True)
    
    print("="*60)
    print("Feature Extraction from Raw Mocap Data")
    print("="*60)
    print(f"Input directory: {raw_files_dir}")
    print(f"Output directory: {processed_files_dir}")
    print(f"Velocity threshold for standing: {velocity_threshold}")
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
                df = get_features(raw_full_path, processed_full_path, velocity_threshold)
                all_processed_dfs.append(df)
            except Exception as e:
                print(f"  ERROR processing {file_name}: {e}")
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
        print(f"\nOverall gait distribution:")
        gait_counts = combined_df['gait_type'].value_counts()
        for gait, count in gait_counts.items():
            print(f"  {gait}: {count} frames ({count/len(combined_df)*100:.1f}%)")
        print(f"\nCombined dataset saved to: {combined_path}")
        print("="*60)
    else:
        print("No files were successfully processed.")