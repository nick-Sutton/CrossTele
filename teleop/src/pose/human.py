from collections import deque
import copy
import numpy as np


class Human:
    def __init__(self, sampling_freq, history_size = 60):
        self.dt = 1.0 / sampling_freq

        self.history = deque(maxlen=history_size)
        self.vel_threshold = 0.2

        self.curr_pose = {}
        self.prev_pose = {}

        self.curr_twist = {}
        self.prev_twist = {}

        # NEW: Track foot contact events for timing
        self.last_left_contact_frame = 0
        self.last_right_contact_frame = 0
        self.frames_since_last_contact = 0

    def calc_contact_probability(self, vel, height, vel_threshold=0.3, height_threshold=0.05):
        vel_mag = np.linalg.norm(vel)

        # contact probability (0=moving, 1=stationary)
        vel_contact = 1.0 - np.clip(vel_mag / vel_threshold, 0, 1)

        # Height-based contact probability (0=high, 1=on ground)
        height_contact = 1.0 - np.clip(height / height_threshold, 0, 1)

        contact_prob = (vel_contact + height_contact) / 2

        return np.clip(contact_prob, 0 ,1)
    
    def extract_gait_features(self, features, frame_idx):
        left_pos = np.array([self.curr_pose["LFoot"].positionX, self.curr_pose["LFoot"].positionY, self.curr_pose["LFoot"].positionZ])
        right_pos = np.array([self.curr_pose["RFoot"].positionX, self.curr_pose["RFoot"].positionY, self.curr_pose["RFoot"].positionZ])
        root_pos = np.array([self.curr_pose["Root"].positionX, self.curr_pose["Root"].positionY, self.curr_pose["Root"].positionZ])

        # determine foot positions relative to root
        min_foot_z = min(left_pos[2], right_pos[2])
        left_height = left_pos[2] - min_foot_z
        right_height = right_pos[2] - min_foot_z

        left_pos_rel = left_pos - root_pos
        right_pos_rel = right_pos - root_pos

        #determine step values
        step_length = np.abs(left_pos[0] - right_pos[0])
        step_width = np.abs(left_pos[1] - right_pos[1])
        step_height = np.abs(left_pos_rel[2] - right_pos_rel[2])

        # calculate velocities for each rigid body
        left_lv = self.curr_twist["LFoot"].linear_velocity
        right_lv = self.curr_twist["RFoot"].linear_velocity
        root_lv = self.curr_twist["Root"].linear_velocity

        root_av = self.curr_twist['Root'].angular_velocity

        # 2) determine movement direction
        # determine foot contact events
        left_contact_prob = self.calc_contact_probability(left_lv, left_height)
        right_contact_prob = self.calc_contact_probability(right_lv, right_height)

        # NEW: Track foot contacts for timing
        contact_threshold = 0.7
        if left_contact_prob > contact_threshold:
            self.last_left_contact_frame = frame_idx
        if right_contact_prob > contact_threshold:
            self.last_right_contact_frame = frame_idx
        
        # Calculate frames since last contact (either foot)
        self.frames_since_last_contact = frame_idx - max(
            self.last_left_contact_frame, 
            self.last_right_contact_frame
        )

        # Kinematic features
        features['root_position_x'] = root_pos[0]
        features['root_position_y'] = root_pos[1]
        features['root_position_z'] = root_pos[2]
        features['root_orientation_x'] = self.curr_pose['Root'].orientationX
        features['root_orientation_y'] = self.curr_pose['Root'].orientationY
        features['root_orientation_z'] = self.curr_pose['Root'].orientationZ
        features['root_linear_velocity_x'] = root_lv[0]
        features['root_linear_velocity_y'] = root_lv[1]
        features['root_linear_velocity_z'] = root_lv[2]
        features['root_angular_velocity_x'] = root_av[0]
        features['root_angular_velocity_y'] = root_av[1]
        features['root_angular_velocity_z'] = root_av[2]
        features['left_linear_velocity_x'] = left_lv[0]
        features['left_linear_velocity_y'] = left_lv[1]
        features['left_linear_velocity_z'] = left_lv[2]
        features['right_linear_velocity_x'] = right_lv[0]
        features['right_linear_velocity_y'] = right_lv[1]
        features['right_linear_velocity_z'] = right_lv[2]

        # Foot Contact and Gait features
        features['left_pos_rel_x'] = left_pos_rel[0]
        features['left_pos_rel_y'] = left_pos_rel[1]
        features['left_pos_rel_z'] = left_pos_rel[2]
        features['right_pos_rel_x'] = right_pos_rel[0]
        features['right_pos_rel_y'] = right_pos_rel[1]
        features['right_pos_rel_z'] = right_pos_rel[2]
        features['step_length'] = step_length
        features['step_width'] = step_width
        features['step_height'] = step_height
        features['left_contact_prob'] = left_contact_prob
        features['right_contact_prob'] = right_contact_prob

        if left_contact_prob > 0.5 and right_contact_prob > 0.5:
            features['support_type'] = 'double'
        elif left_contact_prob <= 0.5 and right_contact_prob <= 0.5:
            features['support_type'] = 'flight'
        else:
            features['support_type'] = 'single'

        features['max_foot_height'] = max(left_pos_rel[2], right_pos_rel[2])

        
        #print("---------------------------------------------------------------------------")
        #print(f'Left Contact: {left_contact_prob}, Right Contact: {right_contact_prob}')
        #print(f'timestep: {self.curr_pose["Root"].timestep}, gait phase: {self.curr_gait_phase}')
        #print("---------------------------------------------------------------------------")

        #Store features in history
        self.history.append(features)
        #self.prev_gait_phase = self.curr_gait_phase

    def update_pose(self):
        self.prev_pose = copy.deepcopy(self.curr_pose)

    def update_twist(self):
        self.prev_twist = copy.deepcopy(self.curr_twist)

    # A peak exists where the lsat velocity was constant zero and the next is decreasing
    # https://stackoverflow.com/questions/22583391/peak-signal-detection-in-realtime-timeseries-data
    # https://www.geeksforgeeks.org/data-analysis/peak-signal-detection-in-real-time-time-series-data/
    # We might want to smooth our data