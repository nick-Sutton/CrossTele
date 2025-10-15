import numpy as np
from sklearn.preprocessing import StandardScaler

class GaitDataPreprocessor:
    def __init__(self):
        self.scalers = {}
        
    def fit(self, features):
        """Fit scalers on training data"""
        for feature in features.columns:
            if feature not in ['frame_id', 'timestamp', 'gait_type']:
                scaler = StandardScaler()
                scaler.fit(features[feature].values.reshape(-1, 1))
                self.scalers[feature] = scaler
    
    def transform(self, features):
        """Transform features using fitted scalers"""
        scaled_features = features.copy()
        for feature, scaler in self.scalers.items():
            scaled_features[feature] = scaler.transform(
                features[feature].values.reshape(-1, 1)
            ).flatten()
        return scaled_features
    
    def create_sequences(self, data, sequence_length=60, stride=10):
        """Create sliding window sequences"""
        sequences = []
        labels = []
        
        # Group by gait type or use continuous sequences
        for gait_type in data['gait_type'].unique():
            gait_data = data[data['gait_type'] == gait_type]
            
            features = gait_data.drop(['frame_id', 'timestamp', 'gait_type'], axis=1)
            for i in range(0, len(features) - sequence_length, stride):
                sequence = features.iloc[i:i+sequence_length].values
                sequences.append(sequence)
                labels.append(gait_type)
                
        return np.array(sequences), np.array(labels)

