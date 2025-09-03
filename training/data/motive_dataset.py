import os
from torch.utils.data import Dataset
import pandas as pd

class MotiveDataset(Dataset):
    def __init__(self, dir, transform=None, target_transform=None):
        self.dir = dir
        self.transform = transform
        self.target_transform = target_transform

    def __len__(self):
        return len(self.img_labels)

    def __getitem__(self, idx):
        pass
    
    def process_data(self, dir):
        df = pd.read_csv(dir)
