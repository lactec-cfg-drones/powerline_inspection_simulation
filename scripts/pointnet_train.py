import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

DATASET_PATH = "../data/dataset_pointnet"


# dataset definition 
class TowerDataset(Dataset):
    def __init__(self, root_dir, npoints=1024):
        self.npoints = npoints
        self.root_dir = root_dir
        self.datapath = []

        search_dirs = [root_dir, os.path.join(root_dir, '0')]
        for d in search_dirs:
            if os.path.exists(d):
                for f in os.listdir(d):
                    if f.endswith('.npy'):
                        self.datapath.append((os.path.join(d, f), 0))
        
        if len(self.datapath) == 0:
            raise RuntimeError(f"No .npy files found in {root_dir}")





    def __len__(self):
        return len(self.datapath)




    def __getitem__(self, index):
        fn, label = self.datapath[index]
        points = np.load(fn)
        # Resampling
        choice = np.random.choice(len(points), self.npoints, replace=True)
        points = points[choice, :]
        points = torch.from_numpy(points).float()
        points = points.transpose(1, 0) # (3, 1024)
        return points, label





# Pointnet++ architecture 
class PointNetSetAbstraction(nn.Module):
    def __init__(self, in_channel, mlp, group_all):
        super(PointNetSetAbstraction, self).__init__()
        self.mlp_convs = nn.ModuleList()
        self.mlp_bns = nn.ModuleList()
        last_channel = in_channel
        for out_channel in mlp:
            self.mlp_convs.append(nn.Conv2d(last_channel, out_channel, 1))
            self.mlp_bns.append(nn.BatchNorm2d(out_channel))
            last_channel = out_channel
        self.group_all = group_all



    def forward(self, points):
        # points shape: [B, D, N]
        new_points = points.unsqueeze(-1) # [B, D, N, 1]

        for i, conv in enumerate(self.mlp_convs):
            bn = self.mlp_bns[i]
            new_points = F.relu(bn(conv(new_points)))

        new_points = torch.max(new_points, 2)[0] # Global Max Pooling
        return new_points






class PointNet2Cls(nn.Module):
    def __init__(self, num_class=2):
        super(PointNet2Cls, self).__init__()
       
        self.sa1 = PointNetSetAbstraction(in_channel=3, mlp=[64, 64, 128], group_all=False)
        self.sa2 = PointNetSetAbstraction(in_channel=128, mlp=[128, 128, 256], group_all=False)
        self.sa3 = PointNetSetAbstraction(in_channel=256, mlp=[256, 512, 1024], group_all=True)
        
        self.fc1 = nn.Linear(1024, 512)
        self.bn1 = nn.BatchNorm1d(512)
        self.fc2 = nn.Linear(512, 256)
        self.bn2 = nn.BatchNorm1d(256)
        self.fc3 = nn.Linear(256, num_class)



    def forward(self, x):
        # x: [B, 3, 1024]
        x = self.sa1(x)
        x = self.sa2(x)
        x = self.sa3(x)
        
        x = x.view(-1, 1024)
        x = F.relu(self.bn1(self.fc1(x)))
        x = F.relu(self.bn2(self.fc2(x)))
        x = self.fc3(x)
        return F.log_softmax(x, -1)





# trainning loop
def run_training():
    EPOCHS = 50
    BATCH_SIZE = 4
    DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


    print(f"--- Starting trainning at: {DEVICE} ---")


    dataset = TowerDataset(root_dir=DATASET_PATH, npoints=1024)
    loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)


    model = PointNet2Cls(num_class=2).to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)


    for epoch in range(EPOCHS):
        model.train()
        total_loss = 0
        for points, labels in loader:
            points, labels = points.to(DEVICE), labels.to(DEVICE)
            optimizer.zero_grad()
            pred = model(points)
            loss = F.nll_loss(pred, labels)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
        
        if (epoch + 1) % 10 == 0:
            print(f"Epoch [{epoch+1}/{EPOCHS}] - Loss: {total_loss/len(loader):.4f}")

    torch.save(model.state_dict(), 'tower_pointnet2_weights.pth')
    print("\n Workout finished! Weights saved.")






if __name__ == '__main__':
    run_training()