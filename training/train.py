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
        self.feature_names = None
        
    def prepare_features(self, df, feature_names):
        """Prepare features by handling categorical variables and scaling"""
        # Create copy of features
        features = df[feature_names].copy()
        
        # Handle support_type (one-hot encode if categorical)
        if 'support_type' in features.columns:
            if features['support_type'].dtype == 'object' or features['support_type'].nunique() < 10:
                support_dummies = pd.get_dummies(features['support_type'], prefix='support')
                features = pd.concat([features.drop('support_type', axis=1), support_dummies], axis=1)
        
        self.feature_names = [col for col in features.columns]
        return features
    
    def create_sequences(self, features, targets, sequence_length=60, stride=10):
        """Create sliding window sequences from time series data"""
        sequences = []
        sequence_labels = []
        
        for i in range(0, len(features) - sequence_length + 1, stride):
            sequence = features.iloc[i:i+sequence_length].values
            # Use the label from the last frame of the sequence
            label = targets.iloc[i+sequence_length-1]
            
            sequences.append(sequence)
            sequence_labels.append(label)
                
        return np.array(sequences), np.array(sequence_labels)
    
    def fit_transform(self, df, feature_names, sequence_length=60, stride=10):
        """Complete preprocessing pipeline"""
        # Prepare features
        features = self.prepare_features(df, feature_names)
        
        # Scale features
        scaled_features = self.scaler.fit_transform(features)
        scaled_features_df = pd.DataFrame(scaled_features, columns=self.feature_names)
        
        # Encode labels
        encoded_labels = self.label_encoder.fit_transform(df['gait_type'])
        encoded_labels_series = pd.Series(encoded_labels, index=df.index)
        
        # Create sequences
        X_sequences, y_sequences = self.create_sequences(
            scaled_features_df, encoded_labels_series, sequence_length, stride
        )
        
        return X_sequences, y_sequences
    
    def transform(self, df, sequence_length=60, stride=10):
        """Transform new data using fitted preprocessor"""
        features = self.prepare_features(df, self.feature_names)
        scaled_features = self.scaler.transform(features)
        scaled_features_df = pd.DataFrame(scaled_features, columns=self.feature_names)
        
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

def train_model(df, feature_names):
    # Setup device
    device = setup_device()
    
    # Debug data first
    #X_train, y_train = debug_data_pipeline(df, feature_names)
    
    # 1. Initialize preprocessor
    preprocessor = GaitDataPreprocessor()
    
    # 2. Better data splitting (shuffle first)
    df_shuffled = df.sample(frac=1, random_state=42).reset_index(drop=True)
    split_idx = int(0.8 * len(df_shuffled))
    train_df = df_shuffled.iloc[:split_idx]
    val_df = df_shuffled.iloc[split_idx:]
    
    # 3. Preprocess data
    X_train, y_train = preprocessor.fit_transform(train_df, feature_names, sequence_length=60, stride=10)
    X_val, y_val = preprocessor.transform(val_df, sequence_length=60, stride=10)
    
    print(f"Training sequences: {X_train.shape}")
    print(f"Validation sequences: {X_val.shape}")
    
    # 4. Convert to tensors
    X_train_tensor = torch.FloatTensor(X_train).to(device)
    y_train_tensor = torch.LongTensor(y_train).to(device)
    X_val_tensor = torch.FloatTensor(X_val).to(device)
    y_val_tensor = torch.LongTensor(y_val).to(device)
    
    # 5. Create dataloaders
    train_loader = DataLoader(TensorDataset(X_train_tensor, y_train_tensor), 
                             batch_size=32, shuffle=True)
    val_loader = DataLoader(TensorDataset(X_val_tensor, y_val_tensor), 
                           batch_size=32, shuffle=False)
    
    # 6. Simpler model architecture
    model = GaitTCN(
        num_features=X_train.shape[2],
        num_classes=len(preprocessor.label_encoder.classes_),
        num_channels=[32, 64, 128],  # Smaller architecture
        kernel_size=3,  # Smaller kernel
        dropout=0.1  # Less dropout
    ).to(device)
    
    # 7. More conservative training setup
    optimizer = optim.AdamW(model.parameters(), lr=1e-4, weight_decay=1e-4)  # Lower LR
    criterion = nn.CrossEntropyLoss()
    
    # 8. Learning rate scheduler
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', 
                                                   patience=5, factor=0.5, verbose=True)
    
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