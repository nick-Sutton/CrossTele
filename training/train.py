import glob
import os
import torch
import torch.nn as nn
import numpy as np
import pandas as pd
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import StandardScaler, LabelEncoder

class GaitDataPreprocessor:
    def __init__(self):
        self.scaler = StandardScaler()
        self.label_encoder = LabelEncoder()
        self.support_encoder = LabelEncoder()
        self.original_feature_names = None
        self.processed_feature_names = None
        self.is_fitted = False
        
        # PRE-DEFINE ALL POSSIBLE LABELS
        known_gait_types = ['walk', 'jog', 'stand'] 
        known_support_types = ['double', 'single', 'flight']
        
        # Fit encoders with known classes upfront - ONLY ONCE
        self.label_encoder.fit(known_gait_types)
        self.support_encoder.fit(known_support_types)
        
        print(f"Predefined gait types: {list(self.label_encoder.classes_)}")
        print(f"Predefined support types: {list(self.support_encoder.classes_)}")
    
    def prepare_features(self, df, feature_names, is_training=False):
        """Prepare features with predefined encoders"""
        available_features = [f for f in feature_names if f in df.columns]
        features = df[available_features].copy()
        
        # Handle support_type with predefined encoder - ONLY TRANSFORM
        if 'support_type' in features.columns:
            if features['support_type'].dtype == 'object':
                # Transform using predefined encoder - NO FITTING
                features['support_type_encoded'] = self.support_encoder.transform(features['support_type'])
                features = features.drop('support_type', axis=1)
        
        self.processed_feature_names = [col for col in features.columns]
        return features
    
    def create_sequences(self, features, targets, sequence_length=60, stride=10):
        """Create sequences that maintain temporal continuity within takes"""
        sequences = []
        sequence_labels = []
        
        for i in range(0, len(features) - sequence_length + 1, stride):
            sequence = features.iloc[i:i+sequence_length].values
            
            # Get the most common label in the sequence
            sequence_labels_in_window = targets.iloc[i:i+sequence_length]
            label = sequence_labels_in_window.mode()[0]
            
            sequences.append(sequence)
            sequence_labels.append(label)
                
        return np.array(sequences), np.array(sequence_labels)
    
    def fit_transform(self, df, feature_names, sequence_length=60, stride=10):
        """Fit scaler only - labels are already predefined"""
        self.is_fitted = True
        self.original_feature_names = feature_names
        
        features = self.prepare_features(df, feature_names, is_training=True)
        
        # Fit scaler only (labels are already encoded)
        scaled_features = self.scaler.fit_transform(features)
        scaled_features_df = pd.DataFrame(scaled_features, columns=self.processed_feature_names)
        
        # Transform gait labels using predefined encoder
        encoded_labels = self.label_encoder.transform(df['gait_type'])
        encoded_labels_series = pd.Series(encoded_labels, index=df.index)
        
        # Create sequences
        X_sequences, y_sequences = self.create_sequences(
            scaled_features_df, encoded_labels_series, sequence_length, stride
        )
        
        print(f"Processed {len(X_sequences)} sequences with {len(self.processed_feature_names)} features")
        return X_sequences, y_sequences
    
    def transform(self, df, sequence_length=60, stride=10):
        """Transform new data using fitted preprocessors"""
        if not self.is_fitted:
            raise ValueError("Preprocessor must be fitted before transform")
        
        features = self.prepare_features(df, self.original_feature_names, is_training=False)
        scaled_features = self.scaler.transform(features)
        scaled_features_df = pd.DataFrame(scaled_features, columns=self.processed_feature_names)
        
        # Transform gait labels using predefined encoder
        encoded_labels = self.label_encoder.transform(df['gait_type'])
        encoded_labels_series = pd.Series(encoded_labels, index=df.index)
        
        X_sequences, y_sequences = self.create_sequences(
            scaled_features_df, encoded_labels_series, sequence_length, stride
        )
        
        return X_sequences, y_sequences
    
class TemporalBlock(nn.Module):
    """A single TCN residual block."""
    def __init__(self, in_channels, out_channels, kernel_size, dilation, dropout):
        super().__init__()
        padding = (kernel_size - 1) * dilation
        
        self.conv1 = nn.Conv1d(in_channels, out_channels, kernel_size,
                              padding=padding, dilation=dilation)
        self.bn1 = nn.BatchNorm1d(out_channels)
        
        self.conv2 = nn.Conv1d(out_channels, out_channels, kernel_size,
                              padding=padding, dilation=dilation)
        self.bn2 = nn.BatchNorm1d(out_channels)
        
        self.dropout = nn.Dropout(dropout)
        self.downsample = (nn.Conv1d(in_channels, out_channels, 1)
                          if in_channels != out_channels else None)
        
        self.init_weights()

    def init_weights(self):
        for m in [self.conv1, self.conv2]:
            nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
            nn.init.constant_(m.bias, 0.0)

    def forward(self, x):
        residual = x
        
        out = self.conv1(x)
        out = self.bn1(out)
        out = F.relu(out)
        out = self.dropout(out)
        
        out = self.conv2(out)
        out = self.bn2(out)
        out = F.relu(out)
        out = self.dropout(out)
        
        # Causal trim
        out = out[:, :, :x.size(2)]
        
        if self.downsample is not None:
            residual = self.downsample(residual)
            residual = residual[:, :, :x.size(2)]
            
        return F.relu(out + residual)


class GaitTCN(nn.Module):
    """
    Temporal Convolutional Network for real-time gait classification.

    Args:
        num_features: Number of input features per timestep.
        num_classes: Number of gait classes to predict.
        num_channels: List defining the number of channels per TCN layer.
        kernel_size: Convolution kernel size.
        dropout: Dropout rate for regularization.
    """
    def __init__(self, num_features, num_classes,
                 num_channels=(64, 64, 128, 128, 256, 256),
                 kernel_size=5, dropout=0.2):
        super().__init__()

        # Initial feature projection
        self.input_proj = nn.Sequential(
            nn.Conv1d(num_features, num_channels[0], 1),
            nn.BatchNorm1d(num_channels[0]),
            nn.ReLU(),
            nn.Dropout(dropout)
        )

        # TCN layers
        layers = []
        in_channels = num_channels[0]
        for i, out_channels in enumerate(num_channels):
            dilation = 2 ** i
            layers.append(
                TemporalBlock(in_channels, out_channels, 
                                   kernel_size, dilation, dropout)
            )
            in_channels = out_channels

        self.tcn = nn.Sequential(*layers)
        
        # Enhanced classifier
        self.classifier = nn.Sequential(
            nn.AdaptiveAvgPool1d(1),
            nn.Flatten(),
            nn.Linear(num_channels[-1], 256),
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(256, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(128, num_classes)
        )

    def forward(self, x):
        x = x.transpose(1, 2)  # [B, F, T]
        x = self.input_proj(x)
        x = self.tcn(x)
        logits = self.classifier(x)
        return logits

def setup_device():
    """Setup device and print GPU info"""
    if torch.cuda.is_available():
        device = torch.device('cuda')
        print(f"🚀 Using GPU: {torch.cuda.get_device_name()}")
        print(f"🚀 GPU Memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
    else:
        device = torch.device('cpu')
        print("⚠️ Using CPU - training will be slower")
    return device

def train_model(data_dir, feature_names, train_ratio=0.8):
    # Setup device
    device = setup_device()
    
    # Get all take files
    take_files = glob.glob(os.path.join(data_dir, "*.csv"))
    print(f"Found {len(take_files)} take files")
    
    # Shuffle and split take files
    np.random.shuffle(take_files)
    split_idx = int(len(take_files) * 0.8)
    train_files = take_files[:split_idx]
    val_files = take_files[split_idx:]
    
    print(f"Training takes: {len(train_files)}")
    print(f"Validation takes: {len(val_files)}")
    
    # Initialize preprocessor 
    preprocessor = GaitDataPreprocessor()
    
    # Process training files (single pass)
    all_train_sequences = []
    all_train_labels = []
    
    for i, train_file in enumerate(train_files):
        df = pd.read_csv(train_file)
        X_take, y_take = preprocessor.fit_transform(df, feature_names)  # Fits SCALER only
        all_train_sequences.append(X_take)
        all_train_labels.append(y_take)
    
    # Process validation files
    all_val_sequences = []
    all_val_labels = []
    
    for i, val_file in enumerate(val_files):
        df = pd.read_csv(val_file)
        X_take, y_take = preprocessor.transform(df)  # Uses fitted scaler & encoders
        all_val_sequences.append(X_take)
        all_val_labels.append(y_take)
    
    # Combine all sequences
    X_train = np.vstack(all_train_sequences) if all_train_sequences else np.array([])
    y_train = np.hstack(all_train_labels) if all_train_labels else np.array([])
    X_val = np.vstack(all_val_sequences) 
    y_val = np.hstack(all_val_labels)
    
    print(f"Final training sequences: {X_train.shape}")
    print(f"Final validation sequences: {X_val.shape}")

    
    # 7. Convert to tensors and train (same as before)
    X_train_tensor = torch.FloatTensor(X_train).to(device)
    y_train_tensor = torch.LongTensor(y_train).to(device)
    X_val_tensor = torch.FloatTensor(X_val).to(device)
    y_val_tensor = torch.LongTensor(y_val).to(device)
    
    train_loader = DataLoader(TensorDataset(X_train_tensor, y_train_tensor), 
                             batch_size=32, shuffle=True)
    val_loader = DataLoader(TensorDataset(X_val_tensor, y_val_tensor), 
                           batch_size=32, shuffle=False)
    
    # 8. Initialize and train model
    model = GaitTCN(
        num_features=X_train.shape[2],
        num_classes=len(preprocessor.label_encoder.classes_),
        num_channels=[32, 64, 128],
        kernel_size=3,
        dropout=0.1
    ).to(device)
    
    # 7. More conservative training setup
    optimizer = optim.AdamW(model.parameters(), lr=1e-4, weight_decay=1e-4)  # Lower LR
    criterion = nn.CrossEntropyLoss()
    
    # 8. Learning rate scheduler
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', 
                                                   patience=5, factor=0.5)
    
    # 9. Training loop with gradient monitoring
    best_val_loss = float('inf')
    train_losses, val_losses, val_accuracies = [], [], []
    
    for epoch in range(100):
        model.train()
        train_loss = 0.0
        
        for batch_X, batch_y in train_loader:
            optimizer.zero_grad()
            outputs = model(batch_X)
            loss = criterion(outputs, batch_y)
            loss.backward()
            
            # More aggressive gradient clipping
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=0.5)
            
            optimizer.step()
            train_loss += loss.item()
        
        # Validation
        model.eval()
        val_loss = 0.0
        correct = 0
        total = 0
        
        with torch.no_grad():
            for batch_X, batch_y in val_loader:
                outputs = model(batch_X)
                loss = criterion(outputs, batch_y)
                val_loss += loss.item()
                
                _, predicted = torch.max(outputs, 1)
                total += batch_y.size(0)
                correct += (predicted == batch_y).sum().item()
        
        # Calculate metrics
        train_loss_avg = train_loss / len(train_loader)
        val_loss_avg = val_loss / len(val_loader)
        val_accuracy = 100 * correct / total
        
        # Update scheduler
        scheduler.step(val_loss_avg)
        
        # Store metrics
        train_losses.append(train_loss_avg)
        val_losses.append(val_loss_avg)
        val_accuracies.append(val_accuracy)
        
        # Print progress every epoch initially
        if epoch % 5 == 0 or epoch < 10:
            current_lr = optimizer.param_groups[0]['lr']
            print(f'Epoch {epoch:3d}: LR={current_lr:.2e}, '
                  f'Train Loss: {train_loss_avg:.4f}, '
                  f'Val Loss: {val_loss_avg:.4f}, '
                  f'Val Acc: {val_accuracy:.2f}%')
        
        # Save best model
        if val_loss_avg < best_val_loss:
            best_val_loss = val_loss_avg
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_loss': val_loss_avg,
                'val_accuracy': val_accuracy,
            }, 'best_gait_model.pth')
    
    return model, preprocessor, train_losses, val_losses, val_accuracies