import glob
import os
import torch
import torch.nn as nn
import numpy as np
import pandas as pd
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from sklearn.preprocessing import StandardScaler, LabelEncoder

class GaitDataPreprocessor:
    def __init__(self):
        self.scaler = StandardScaler()
        self.label_encoder = LabelEncoder()
        self.support_encoder = LabelEncoder()
        self.original_feature_names = None
        self.processed_feature_names = None
        self.is_fitted = False
        
        # Predefined labels
        known_gait_types = ['walk', 'jog', 'stand'] 
        known_support_types = ['double', 'single', 'flight']
        
        self.label_encoder.fit(known_gait_types)
        self.support_encoder.fit(known_support_types)
        
        print(f"Predefined gait types: {list(self.label_encoder.classes_)}")
        print(f"Predefined support types: {list(self.support_encoder.classes_)}")
    
    def prepare_features(self, df, feature_names):
        """Prepare features - encode categorical variables"""
        available_features = [f for f in feature_names if f in df.columns]
        features = df[available_features].copy()
        
        # Encode support_type
        if 'support_type' in features.columns:
            if features['support_type'].dtype == 'object':
                features['support_type_encoded'] = self.support_encoder.transform(features['support_type'])
                features = features.drop('support_type', axis=1)
        
        self.processed_feature_names = list(features.columns)
        return features
    
    def create_sequences_from_files(self, file_list, feature_names, sequence_length=60, stride=30):
        """Create sequences maintaining take boundaries"""
        all_sequences = []
        all_labels = []
        
        for file_path in file_list:
            df = pd.read_csv(file_path)
            features = self.prepare_features(df, feature_names)
            labels = self.label_encoder.transform(df['gait_type'])
            
            # Create sequences within this take only
            for i in range(0, len(features) - sequence_length + 1, stride):
                sequence = features.iloc[i:i+sequence_length].values
                # Use mode of labels in sequence
                label_window = labels[i:i+sequence_length]
                label = np.bincount(label_window).argmax()
                
                all_sequences.append(sequence)
                all_labels.append(label)
        
        return np.array(all_sequences), np.array(all_labels)
    
    def fit_transform(self, train_files, feature_names, sequence_length=60, stride=30):
        """Fit scaler on ALL training data, then create sequences"""
        self.is_fitted = True
        self.original_feature_names = feature_names
        
        # Step 1: Load ALL training data and concatenate
        all_train_features = []
        for file_path in train_files:
            df = pd.read_csv(file_path)
            features = self.prepare_features(df, feature_names)
            all_train_features.append(features)
        
        all_train_features = pd.concat(all_train_features, ignore_index=True)
        
        # Step 2: Fit scaler on ALL training data
        print(f"Fitting scaler on {len(all_train_features)} total training samples")
        self.scaler.fit(all_train_features)
        
        # Step 3: Create sequences from each file with fitted scaler
        X_sequences, y_sequences = self.create_sequences_from_files(
            train_files, feature_names, sequence_length, stride
        )
        
        # Step 4: Scale the sequences
        n_samples, seq_len, n_features = X_sequences.shape
        X_flat = X_sequences.reshape(-1, n_features)
        X_scaled = self.scaler.transform(X_flat)
        X_sequences = X_scaled.reshape(n_samples, seq_len, n_features)
        
        print(f"Created {len(X_sequences)} training sequences with {n_features} features")
        print(f"Sequence shape: {X_sequences.shape}")
        
        return X_sequences, y_sequences
    
    def transform(self, val_files, sequence_length=60, stride=30):
        """Transform validation data using fitted scaler"""
        if not self.is_fitted:
            raise ValueError("Preprocessor must be fitted before transform")
        
        # Create sequences
        X_sequences, y_sequences = self.create_sequences_from_files(
            val_files, self.original_feature_names, sequence_length, stride
        )
        
        # Scale the sequences
        n_samples, seq_len, n_features = X_sequences.shape
        X_flat = X_sequences.reshape(-1, n_features)
        X_scaled = self.scaler.transform(X_flat)
        X_sequences = X_scaled.reshape(n_samples, seq_len, n_features)
        
        print(f"Created {len(X_sequences)} validation sequences")
        
        return X_sequences, y_sequences


class TemporalBlock(nn.Module):
    """TCN residual block with weight normalization"""
    def __init__(self, in_channels, out_channels, kernel_size, dilation, dropout):
        super().__init__()
        padding = (kernel_size - 1) * dilation
        
        self.conv1 = nn.utils.weight_norm(
            nn.Conv1d(in_channels, out_channels, kernel_size,
                     padding=padding, dilation=dilation)
        )
        self.conv2 = nn.utils.weight_norm(
            nn.Conv1d(out_channels, out_channels, kernel_size,
                     padding=padding, dilation=dilation)
        )
        
        self.dropout = nn.Dropout(dropout)
        self.downsample = (nn.utils.weight_norm(nn.Conv1d(in_channels, out_channels, 1))
                          if in_channels != out_channels else None)
        
        self.relu = nn.ReLU()

    def forward(self, x):
        residual = x
        
        # First convolution
        out = self.conv1(x)
        out = self.relu(out)
        out = self.dropout(out)
        
        # Second convolution
        out = self.conv2(out)
        out = self.relu(out)
        out = self.dropout(out)
        
        # Causal truncation
        out = out[:, :, :x.size(2)]
        
        # Residual connection
        if self.downsample is not None:
            residual = self.downsample(residual)
            residual = residual[:, :, :x.size(2)]
            
        return self.relu(out + residual)


class GaitTCN(nn.Module):
    """Simplified TCN for gait classification"""
    def __init__(self, num_features, num_classes,
                 num_channels=[64, 128, 256],
                 kernel_size=7, dropout=0.3):
        super().__init__()

        # Input projection
        self.input_proj = nn.Sequential(
            nn.Conv1d(num_features, num_channels[0], 1),
            nn.ReLU(),
            nn.Dropout(dropout)
        )

        # TCN blocks
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
        
        # Simplified classifier
        self.classifier = nn.Sequential(
            nn.AdaptiveAvgPool1d(1),
            nn.Flatten(),
            nn.Linear(num_channels[-1], num_classes)
        )

    def forward(self, x):
        # x: [batch, seq_len, features]
        x = x.transpose(1, 2)  # [batch, features, seq_len]
        x = self.input_proj(x)
        x = self.tcn(x)
        logits = self.classifier(x)
        return logits


def setup_device():
    """Setup device"""
    if torch.cuda.is_available():
        device = torch.device('cuda')
        print(f"🚀 Using GPU: {torch.cuda.get_device_name()}")
    else:
        device = torch.device('cpu')
        print("⚠️ Using CPU")
    return device


def train_model(data_dir, feature_names, sequence_length=60, stride=30):
    device = setup_device()
    
    # Get all files and split
    take_files = sorted(glob.glob(os.path.join(data_dir, "*.csv")))
    print(f"Found {len(take_files)} take files")
    
    np.random.seed(42)
    np.random.shuffle(take_files)
    split_idx = int(len(take_files) * 0.8)
    train_files = take_files[:split_idx]
    val_files = take_files[split_idx:]
    
    print(f"Training takes: {len(train_files)}")
    print(f"Validation takes: {len(val_files)}")
    
    # Initialize preprocessor and fit on training data
    preprocessor = GaitDataPreprocessor()
    X_train, y_train = preprocessor.fit_transform(
        train_files, feature_names, sequence_length, stride
    )
    X_val, y_val = preprocessor.transform(
        val_files, sequence_length, stride
    )
    
    print(f"\nFinal shapes:")
    print(f"Train: {X_train.shape}, {y_train.shape}")
    print(f"Val: {X_val.shape}, {y_val.shape}")
    print(f"Label distribution (train): {np.bincount(y_train)}")
    print(f"Label distribution (val): {np.bincount(y_val)}")
    
    # Convert to tensors
    X_train_tensor = torch.FloatTensor(X_train).to(device)
    y_train_tensor = torch.LongTensor(y_train).to(device)
    X_val_tensor = torch.FloatTensor(X_val).to(device)
    y_val_tensor = torch.LongTensor(y_val).to(device)
    
    train_loader = DataLoader(
        TensorDataset(X_train_tensor, y_train_tensor), 
        batch_size=64, shuffle=True
    )
    val_loader = DataLoader(
        TensorDataset(X_val_tensor, y_val_tensor), 
        batch_size=64, shuffle=False
    )
    
    # Initialize model
    model = GaitTCN(
        num_features=X_train.shape[2],
        num_classes=len(preprocessor.label_encoder.classes_),
        num_channels=[64, 128, 256],
        kernel_size=7,
        dropout=0.3
    ).to(device)
    
    print(f"\nModel parameters: {sum(p.numel() for p in model.parameters()):,}")
    
    # Training setup
    optimizer = optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-5)
    criterion = nn.CrossEntropyLoss()
    scheduler = optim.lr_scheduler.CosineAnnealingWarmRestarts(
        optimizer, T_0=10, T_mult=2
    )
    
    # Training loop
    best_val_acc = 0
    train_losses, val_losses, val_accuracies = [], [], []
    patience_counter = 0
    max_patience = 20
    
    for epoch in range(100):
        # Training
        model.train()
        train_loss = 0.0
        
        for batch_X, batch_y in train_loader:
            optimizer.zero_grad()
            outputs = model(batch_X)
            loss = criterion(outputs, batch_y)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
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
        
        # Metrics
        train_loss_avg = train_loss / len(train_loader)
        val_loss_avg = val_loss / len(val_loader)
        val_accuracy = 100 * correct / total
        
        scheduler.step()
        
        train_losses.append(train_loss_avg)
        val_losses.append(val_loss_avg)
        val_accuracies.append(val_accuracy)
        
        # Print progress
        if epoch % 5 == 0:
            current_lr = optimizer.param_groups[0]['lr']
            print(f'Epoch {epoch:3d}: LR={current_lr:.2e}, '
                  f'Train Loss: {train_loss_avg:.4f}, '
                  f'Val Loss: {val_loss_avg:.4f}, '
                  f'Val Acc: {val_accuracy:.2f}%')
        
        # Save best model and early stopping
        if val_accuracy > best_val_acc:
            best_val_acc = val_accuracy
            patience_counter = 0
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'val_accuracy': val_accuracy,
                'preprocessor': preprocessor,
            }, 'best_gait_model.pth')
            print(f"  ✓ New best model saved (acc: {val_accuracy:.2f}%)")
        else:
            patience_counter += 1
            if patience_counter >= max_patience:
                print(f"\nEarly stopping at epoch {epoch}")
                break
    
    print(f"\nBest validation accuracy: {best_val_acc:.2f}%")
    return model, preprocessor, train_losses, val_losses, val_accuracies