import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

from ComputerVision import CNN
import torch
from PIL import Image
from torchvision import transforms
import matplotlib.pyplot as plt
import json

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

model = CNN()
model_path = "depth_model (1).pth"

state_dict = torch.load(model_path, map_location=device)
model.load_state_dict(state_dict)
model = model.to(device)
model.eval()

transform = transforms.ToTensor()

# test_path = 'Data/Images/01633.jpg' #Checking image from training data (More debugging than testing)
# image = Image.open(test_path).convert("RGB")
# image = transform(image)
# image = image.unsqueeze(0).to(device)

with open("test_filenames.json", "r") as f:
    test_data = json.load(f)

# with torch.no_grad():
#     pred = model(image)



for file in test_data[:10]:
    path = 'Data/Images/{}'.format(file)
    img = Image.open(path).convert("RGB")
    img = transform(img)
    img = img.unsqueeze(0).to(device)
    original = 'Data/datasets_depth/depth_map/{}_depth.png'.format(file[:5])
    original = Image.open(original)

    with torch.no_grad():
        pred = model(img)

    depth = pred.squeeze().cpu().numpy()
    depth = (depth - depth.min()) / (depth.max() - depth.min())
    
    plt.subplot(1,2,1)
    plt.imshow(depth, cmap = 'gray')

    plt.subplot(1,2,2)
    plt.imshow(original)
    plt.colorbar()

    plt.show()



# plt.imshow(depth, cmap="gray")
# plt.colorbar()
# plt.show()
#Re



