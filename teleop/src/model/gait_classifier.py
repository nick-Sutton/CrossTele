from collections import deque
import copy
import numpy as np
import torch
from ctrl_interface.ctrl_interface import CtrlInterface


class GaitClassifier:
    """Real-time gait classification using a sliding window buffer."""
    
    def __init__(self, model_path, sequence_length=60, device=None):
        if device is None:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = torch.device(device)
        
        print(f"Loading gait classifier from {model_path}...")
        checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)
        
        self.preprocessor = checkpoint['preprocessor']
        self.sequence_length = sequence_length
        
        # Import your model class
        from training.train import GaitTCN
        
        num_features = len(self.preprocessor.processed_feature_names)
        num_classes = len(self.preprocessor.label_encoder.classes_)
        
        self.model = GaitTCN(
            num_features=num_features,
            num_classes=num_classes,
            num_channels=[64, 128, 256],
            kernel_size=7,
            dropout=0.3
        ).to(self.device)
        
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.eval()
        
        print(f"✓ Model loaded on {self.device}")
        print(f"  Classes: {self.preprocessor.label_encoder.classes_}")
        print(f"  Buffer size: {sequence_length} frames")
        
        self.frame_buffer = deque(maxlen=sequence_length)
        self.is_ready = False
        
    def _prepare_features(self, raw_features):
        """Convert feature dict to processed array."""
        feature_values = []
        for feature_name in self.preprocessor.original_feature_names:
            if feature_name == 'support_type':
                support_val = raw_features[feature_name]
                encoded_val = self.preprocessor.support_encoder.transform([support_val])[0]
                feature_values.append(encoded_val)
            else:
                feature_values.append(raw_features[feature_name])
        
        feature_array = np.array(feature_values, dtype=np.float32)
        feature_array = self.preprocessor.scaler.transform(feature_array.reshape(1, -1))
        return feature_array.flatten()
    
    def predict(self, raw_features):
        """Make prediction on current frame."""
        # Add frame to buffer
        processed_features = self._prepare_features(raw_features)
        self.frame_buffer.append(processed_features)
        
        if len(self.frame_buffer) == self.sequence_length:
            self.is_ready = True
        
        # Return None if not ready
        if not self.is_ready:
            frames_needed = self.sequence_length - len(self.frame_buffer)
            return None, 0.0, {'buffering': frames_needed}
        
        # Convert buffer to tensor
        sequence = np.array(list(self.frame_buffer))
        sequence_tensor = torch.FloatTensor(sequence).unsqueeze(0).to(self.device)
        
        # Predict
        with torch.no_grad():
            outputs = self.model(sequence_tensor)
            probabilities = torch.softmax(outputs, dim=1)
            confidence, predicted_idx = torch.max(probabilities, 1)
            
            predicted_idx = predicted_idx.item()
            confidence = confidence.item()
        
        prediction = self.preprocessor.label_encoder.inverse_transform([predicted_idx])[0]
        
        prob_dict = {
            name: probabilities[0, idx].item() 
            for idx, name in enumerate(self.preprocessor.label_encoder.classes_)
        }
        
        return prediction, confidence, prob_dict
    
    def reset(self):
        """Clear buffer."""
        self.frame_buffer.clear()
        self.is_ready = False

    def select_robot_command(self, gait_prediction, confidence, frames_since_contact, 
                            target_twist, sampling_freq=240.0):
        """
        Select robot command based on gait prediction and contact timing.
        
        Args:
            gait_prediction: 'walk', 'jog', 'stand', or None (buffering)
            confidence: Prediction confidence (0-1)
            frames_since_contact: Frames since last foot contact
            target_twist: Twist object with linear/angular velocities
            sampling_freq: Sampling frequency in Hz
        """
        # Calculate time since last contact
        time_since_contact = frames_since_contact / sampling_freq
        
        # Default to standing if model is still buffering or low confidence
        if gait_prediction is None or confidence < 0.6:
            return 0.0, 0.0, 0.0
        
        # Base scaling factors for each gait type
        if gait_prediction == 'stand':
            scale = 0.0

            rx = scale * target_twist.angular_velocity[0]
            ry = scale * target_twist.angular_velocity[1]

            CtrlInterface.stand(rx=rx, ry=ry,rz=0)
            
        elif gait_prediction == 'walk':
            # Walking: moderate speed
            # Increase responsiveness during swing phase (no contact)
            if time_since_contact > 0.2:  # Swing phase (>0.2 seconds since contact)
                scale = 1.0
            else:  # Stance phase (recent contact)
                scale = 0.8
            
            vx = scale * target_twist.linear_velocity[0]
            vy = scale * target_twist.linear_velocity[1]
            vrz = 0.0 * target_twist.angular_velocity[2]

            CtrlInterface.walk(vx, vy, vrz)
            
        elif gait_prediction == 'jog':
            # Jogging: higher speed
            # More aggressive scaling during flight phase
            if time_since_contact > 0.15:  # Flight/swing phase
                scale = 1.5
            else:  # Stance phase
                scale = 1.2
            
            vx = scale * target_twist.linear_velocity[0]

            CtrlInterface.bound(vx)
        
        else:
            # Unknown gait: default to standing
            CtrlInterface.stand()
        
        # Velocity limits for safety
        #max_linear_vel = 2.0  # m/s
        #max_angular_vel = 1.0  # rad/s
        
        #vx = np.clip(vx, -max_linear_vel, max_linear_vel)
        #vy = np.clip(vy, -max_linear_vel, max_linear_vel)
        #wz = np.clip(wz, -max_angular_vel, max_angular_vel)