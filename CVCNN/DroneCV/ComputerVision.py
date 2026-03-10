import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset
from PIL import Image
import os
from torchvision import transforms

#Build CNN model - takes full images & outputs respective depth maps

n_epochs = 1
lr = 0.01
batch_size = 32

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)
torch.backends.cudnn.benchmark = True

class CNN(torch.nn.Module):
    def __init__(self):
        super().__init__()
        #encoder
        self.conv1 = torch.nn.Conv2d(in_channels=3,out_channels=64, kernel_size=3, stride=1, padding=1)
        self.conv2 = torch.nn.Conv2d(in_channels=64,out_channels=128, kernel_size=3, stride=1, padding=1)
        self.conv3 = torch.nn.Conv2d(in_channels=128,out_channels=256, kernel_size=3, stride=2, padding=1)
        #decoder
        self.deconv1 = torch.nn.ConvTranspose2d(in_channels=256,out_channels=128,kernel_size=3,stride=2,padding=1)
        self.deconv2 = torch.nn.ConvTranspose2d(in_channels=256,out_channels=64,kernel_size=3,stride=1,padding=1)
        self.deconv3 = torch.nn.ConvTranspose2d(in_channels=128,out_channels=1,kernel_size=3,stride=1,padding=1)

        self.act1 = torch.nn.ReLU()

    def forward(self,x):

        x = self.act1(self.conv1(x))
        cross1 = x
        x = self.act1(self.conv2(x))
        cross2 = x
        x = self.act1(self.conv3(x))

        x = self.act1(self.deconv1(x))
        x = torch.nn.functional.interpolate(x, size=cross2.shape[2:], mode='bilinear', align_corners=False)
        x = torch.cat([x, cross2],dim=1)
        x = self.act1(self.deconv2(x))
        x = torch.nn.functional.interpolate(x, size=cross1.shape[2:], mode='bilinear', align_corners=False)
        x = torch.cat([x, cross1],dim=1)
        x = self.act1(self.deconv3(x))
        y = torch.nn.functional.interpolate(x, size=cross1.shape[2:], mode='bilinear', align_corners=False)

        return y

#Dataloading & Training algorithm

model = CNN().to(device)

if __name__ == '__main__':
    #Create Dataset
    class ImageDataset():
        def __init__(self,image_dir, depth_dir, transform = None):
            self.image_dir = image_dir
            self.depth_dir = depth_dir
            self.image_files = sorted(os.listdir(image_dir))
            self.transform = transform
            
        def __len__(self):
            return len(self.image_files)
        
        def __getitem__(self,idx):
            img_name = self.image_files[idx]
            base = os.path.splitext(img_name)[0]

            image_path = os.path.join(self.image_dir, img_name)
            target_path = os.path.join(self.depth_dir, '{}_depth.png'.format(base))

            image = Image.open(image_path).convert("YCbCr")
            target = Image.open(target_path).convert("L")

            if self.transform:
                image = self.transform(image)
                target = self.transform(target)

            return image, target

    #Dataloading
    transform = transforms.ToTensor()

    dataset = ImageDataset('./Data/Images/','./Data/datasets_depth/depth_map/',transform=transform)

    loader = DataLoader(dataset, batch_size, shuffle=True, pin_memory=True)

    #Train Model
    loss_fn = torch.nn.MSELoss()
    optimiser = torch.optim.Adam(model.parameters(), lr = lr)

    losses = []

    for i in range(n_epochs):
        epoch_loss = 0
        for X_batch, Y_batch in loader:
            X_batch = X_batch.to(device)
            Y_batch = Y_batch.to(device)
            optimiser.zero_grad()
            y_pred = model(X_batch)
            loss = loss_fn(y_pred,Y_batch)
            loss.backward()
            optimiser.step()
            epoch_loss += loss.item()
            print("batch_loss", loss)
        print("Epoch {}/{}: Loss = {}".format(i+1, n_epochs, epoch_loss))

    #Save Model Parameters
    torch.save(model.state_dict(), "depth_model.pth")


