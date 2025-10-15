from util import reformat_data
import pandas as pd
from training.train import train_model

if __name__ == '__main__':
    df = pd.read_csv("./training/dataset/TrainingData/combined_features.csv")
    
    feature_names = [
        'root_position_x', 'root_position_y', 'root_position_z',
        'root_orientation_x', 'root_orientation_y', 'root_orientation_z',
        'root_linear_velocity_x', 'root_linear_velocity_y', 'root_linear_velocity_z',
        'root_angular_velocity_x', 'root_angular_velocity_y', 'root_angular_velocity_z',
        'left_linear_velocity_x', 'left_linear_velocity_y', 'left_linear_velocity_z',
        'right_linear_velocity_x', 'right_linear_velocity_y', 'right_linear_velocity_z',
        'left_pos_rel_x', 'left_pos_rel_y', 'left_pos_rel_z', 
        'right_pos_rel_x', 'right_pos_rel_y', 'right_pos_rel_z',
        'step_length', 'step_width', 'step_height',
        'left_contact_prob', 'right_contact_prob', 'support_type',
        'left_swing_phase', 'right_swing_phase', 'max_foot_height'
    ]

    trained_model, _ = train_model(df, feature_names)

    #reformat_data.generate_reformatted_data()
    #reformat_data.generate_lable_data()
    #reformat_data.generate_feature_data()