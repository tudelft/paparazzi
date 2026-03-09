import os
import glob
import cv2
import numpy as np
from random import randrange
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import TensorDataset, DataLoader

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)  # should print "cuda" if GPU is available

# -----------------
# PARAMETERS
# -----------------
samples_per_image = 75000   # smaller for memory
patch_size = 16           # CNN input patch size
batch_size = 32
n_epochs = 5           # smaller for testing
lr = 0.01

image_folder = './Data/Images/'
depth_folder = './Data/datasets_depth/depth_map'

# -----------------
# CNN MODEL
# -----------------
class CNN(nn.Module):
    def __init__(self, in_channels=3, out_channels=1):
        super().__init__()
        # Encoder
        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, padding=1)
        # Decoder (keep same spatial size)
        self.upconv1 = nn.ConvTranspose2d(128, 64, kernel_size=1, stride=1)
        self.upconv2 = nn.ConvTranspose2d(64, 32, kernel_size=1, stride=1)
        self.out_conv = nn.Conv2d(32, out_channels, kernel_size=1)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.relu(self.upconv1(x))
        x = self.relu(self.upconv2(x))
        y_pred = self.out_conv(x)
        return y_pred
    
model = CNN()

    
if __name__ == '__main__':

    # -----------------
    # DATA LOADING
    # -----------------
    X = []
    Y = []

    images = glob.glob(os.path.join(image_folder, '*.jpg'))
    print(f"Found {len(images)} images.")

    for img_path in images:
        base_name = os.path.basename(img_path)
        name_no_ext = os.path.splitext(base_name)[0]
        depth_path = os.path.join(depth_folder, name_no_ext + '_depth.png')

        if not os.path.exists(depth_path):
            print("Depth map not found:", depth_path)
            continue

        img = cv2.imread(img_path)
        depth_map = cv2.imread(depth_path, cv2.IMREAD_GRAYSCALE)

        h, w, _ = img.shape
        yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)

    for _ in range(samples_per_image):
        x0 = randrange(0, w - patch_size)
        y0 = randrange(0, h - patch_size)

        patch = yuv[y0:y0+patch_size, x0:x0+patch_size, :]
        depth_patch = depth_map[y0:y0+patch_size, x0:x0+patch_size]

        patch = np.transpose(patch, (2, 0, 1))       # (3,H,W)
        depth_patch = np.expand_dims(depth_patch, 0) # (1,H,W)

        X.append(patch)
        Y.append(depth_patch)

    # Convert to tensors
    X_tensor = torch.tensor(X, dtype=torch.float32)
    Y_tensor = torch.tensor(Y, dtype=torch.float32)

    print("X_tensor shape:", X_tensor.shape)
    print("Y_tensor shape:", Y_tensor.shape)

    # -----------------
    # DATASET & DATALOADER
    # -----------------
    dataset = TensorDataset(X_tensor, Y_tensor)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
    # Loss and optimizer
    loss_fn = nn.MSELoss()
    optimiser = torch.optim.Adam(model.parameters(), lr=lr)

    # -----------------
    # TRAINING LOOP
    # -----------------
    losses = []

    for epoch in range(n_epochs):
        epoch_loss = 0
        for X_batch, Y_batch in loader:
            optimiser.zero_grad()
            y_pred = model(X_batch)
            loss = loss_fn(y_pred, Y_batch)
            loss.backward()
            optimiser.step()
            epoch_loss += loss.item()
        losses.append(epoch_loss / len(loader))
        print(f"Epoch {epoch+1}/{n_epochs}, Loss: {losses[-1]:.4f}")

    torch.save(model.state_dict(), "depth_model.pth")
