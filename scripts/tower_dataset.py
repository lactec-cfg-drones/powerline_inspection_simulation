import os
import numpy as np
import torch
from torch.utils.data import Dataset




class TowerDataset(Dataset):
    def __init__(self, root_dir, npoints=1024, train=True):
        self.npoints = npoints
        self.root_dir = root_dir
        self.datapath = []


        # dataset_pointnet/0  <- normal towers 
        # dataset_pointnet/1  <- towers with defect 
        for label in [0, 1]:
            dir_path = os.path.join(self.root_dir, str(label))
            if os.path.exists(dir_path):
                for f in os.listdir(dir_path):
                    if f.endswith('.npy'):
                        self.datapath.append((os.path.join(dir_path, f), label))
        


        if len(self.datapath) == 0:
            for f in os.listdir(self.root_dir):
                if f.endswith('.npy'):
                    self.datapath.append((os.path.join(self.root_dir, f), 0))





    def __len__(self):
        return len(self.datapath)






    def __getitem__(self, index):
        fn, label = self.datapath[index]
        points = np.load(fn)

        # Resampling to ensure exactly N points
        choice = np.random.choice(len(points), self.npoints, replace=True)
        points = points[choice, :]


        # converting to Tensor (Format: Channels x Points)
        # PointNet++ expects the format[3, 1024]
        points = torch.from_numpy(points).float()
        points = points.transpose(1, 0) 

        return points, label