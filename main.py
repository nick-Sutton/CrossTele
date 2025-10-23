import glob
import os
import numpy as np
import torch
from util import reformat_data
from util import data_fixer
import pandas as pd
from training.train import analyze_misclassification, train_model

if __name__ == '__main__':
    
    
    #Train Model
    data_dir = './training/dataset/TrainingData/'
    
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
        'left_contact_prob', 'right_contact_prob', 'support_type', 'max_foot_height'
    ]
    
    model, preprocessor, *history, save_path = train_model(
        data_dir='./training/dataset/TrainingData',
        feature_names=feature_names,
        run_analysis=True,
        save_dir='./models')
    
    '''
    param_grid = {
    'sequence_length': [60, 80, 100],
    'stride': [20, 30, 40],
    'num_channels': [[32, 64, 128], [64, 128, 256], [32, 64, 128, 256]],
    'kernel_size': [5, 7, 9],
    'dropout': [0.3, 0.4, 0.5],
    'learning_rate': [1e-3, 5e-4, 1e-4],
    'weight_decay': [1e-3, 1e-4, 1e-5],
    'batch_size': [32, 64],
    'max_epochs': [100],
    'balance_data': [False, True]
    }

    #results = cross_validate_hyperparam_search(data_dir, feature_names, param_grid, k_folds=5)
    '''


    #reformat_data.generate_reformatted_data()
    #reformat_data.generate_lable_data()
    #reformat_data.generate_feature_data()
    #data_fixer.generate_feature_data()
