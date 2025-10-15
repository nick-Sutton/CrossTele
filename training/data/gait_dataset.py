import torch
from torch.utils.data import Dataset, DataLoader

class GaitDataset(Dataset):
    def __init__(self, X, y, window_size=120, stride=20):
        self.X = X
        self.y = y
        self.window_size = window_size
        self.stride = stride
        self.indices = self._compute_indices()

    def _compute_indices(self):
        indices = []
        for i in range(0, len(self.X) - self.window_size, self.stride):
            indices.append(i)
        return indices

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        i = self.indices[idx]
        window = self.X[i:i+self.window_size]
        label = self.y[i+self.window_size-1]  # label for last timestep
        return torch.tensor(window, dtype=torch.float32), torch.tensor(label, dtype=torch.long)