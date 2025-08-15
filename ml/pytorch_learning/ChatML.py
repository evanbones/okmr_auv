import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from torch.utils.data import Dataset, DataLoader
from PIL import Image
import os
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)
# Custom Dataset for YOLO Format Labels
class YoloBBoxDataset(Dataset):
    def __init__(self, image_folder, label_folder, transform=None):
        self.image_folder = image_folder
        self.label_folder = label_folder
        self.transform = transform
        self.image_files = os.listdir(image_folder)
    
    def __len__(self):
        return len(self.image_files)
    
    def __getitem__(self, idx):
        img_name = self.image_files[idx]
        img_path = os.path.join(self.image_folder, img_name)
        image = Image.open(img_path).convert("RGB")
        label_path = os.path.join(self.label_folder, img_name.split('.')[0] + '.txt')
        bbox = self.load_yolo_bbox(label_path)
        if self.transform:
            image = self.transform(image)
        return image, bbox

    def load_yolo_bbox(self, label_path):
        with open(label_path, 'r') as file:
            first_line = file.readline().strip().split()
            x_center, y_center, width, height = map(float, first_line[1:])
            return torch.tensor([x_center, y_center, width, height], dtype=torch.float32)

# Simple CNN for Bounding Box Regression
class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.pool = nn.MaxPool2d(2, 2)
        self.fc1 = nn.Linear(64 * 56 * 56, 300)  # Assuming 56x56 after pooling
        self.fc2 = nn.Linear(300, 4)
    
    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))
        x = self.pool(torch.relu(self.conv2(x)))
        x = x.view(x.size(0), -1)
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class DeepConvNet(nn.Module):
    def __init__(self):
        super(DeepConvNet, self).__init__()

        # Deeper architecture with more layers
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1)
        self.conv4 = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1)
        
        # MaxPooling
        self.pool = nn.MaxPool2d(2, 2)

        # Adding Batch Normalization after convolutional layers
        self.bn1 = nn.BatchNorm2d(32)
        self.bn2 = nn.BatchNorm2d(64)
        self.bn3 = nn.BatchNorm2d(128)
        self.bn4 = nn.BatchNorm2d(256)

        # Fully connected layers
        self.fc1 = nn.Linear(256 * 14 * 14, 1000)  # Adjust for new size after additional layers
        self.fc2 = nn.Linear(1000, 500)
        self.fc3 = nn.Linear(500, 4)  # Output for 4 coordinates (for bounding box)

    def forward(self, x):
        # Convolutional and pooling layers with ReLU activations
        x = torch.relu(self.bn1(self.conv1(x)))
        x = self.pool(x)
        
        x = torch.relu(self.bn2(self.conv2(x)))
        x = self.pool(x)
        
        x = torch.relu(self.bn3(self.conv3(x)))
        x = self.pool(x)
        
        x = torch.relu(self.bn4(self.conv4(x)))
        x = self.pool(x)

        # Flatten for fully connected layers
        x = x.view(x.size(0), -1)

        # Fully connected layers with ReLU activation
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        
        # Output layer (without ReLU to allow negative outputs if necessary)
        x = self.fc3(x)
        return x

# Load and visualize data
def visualize_data(dataloader):
    images, boxes = next(iter(dataloader))
    for i in range(len(images)):
        img = images[i].permute(1, 2, 0).numpy()
        bbox = boxes[i].numpy()
        h, w, _ = img.shape
        x1 = (bbox[0] - bbox[2] / 2) * w
        y1 = (bbox[1] - bbox[3] / 2) * h
        x2 = (bbox[0] + bbox[2] / 2) * w
        y2 = (bbox[1] + bbox[3] / 2) * h
        fig, ax = plt.subplots(1)
        ax.imshow(img)
        rect = patches.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=2, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
        plt.show()

# Training Loop
def train_model(model, dataloader, epochs=10, lr = 0.001):
    model = model.to(device)
    
    criterion = nn.SmoothL1Loss()
    optimizer = optim.Adam(model.parameters(), lr)
    for epoch in range(epochs):
        model.train()
        lr *= 0.3
        running_loss = 0.0
        for images, labels in dataloader:
            images = images.to(device)
            labels = labels.to(device)
            optimizer.zero_grad()
            outputs = model(images)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
        print(f"Epoch {epoch+1}, Loss: {running_loss / len(dataloader):.4f}")

def load_yolo_bbox(label_path):
        with open(label_path, 'r') as file:
            first_line = file.readline().strip().split()
            x_center, y_center, width, height = map(float, first_line[1:])
            return torch.tensor([x_center, y_center, width, height], dtype=torch.float32)
def load_single_image(image_folder, label_folder, image_name, transform=None):
    img_path = os.path.join(image_folder, image_name)
    image = Image.open(img_path).convert("RGB")

    label_path = os.path.join(label_folder, image_name.split('.')[0] + '.txt')
    
    # Assuming you have a function to load YOLO bounding boxes
    bbox = load_yolo_bbox(label_path)  

    if transform:
        image = transform(image)

    return image, torch.tensor(bbox, dtype=torch.float32)
def visualize_inference(model, test_images, device):
    model.eval()  # Set to evaluation mode
    
    with torch.no_grad():  # No need to track gradients for inference
        test_images = test_images.to(device)  # Make sure the image is on the same device as the model
        output = model(test_images)  # Pass through the model
        
        for i in range(test_images.shape[0]):  # Iterate through the batch of images
            pred_bbox = output[i].cpu().numpy()  # Get bounding box for each image in the batch

            # Assuming output is in YOLO format [x_center, y_center, width, height]
            h, w, _ = test_images[i].permute(1, 2, 0).cpu().numpy().shape
            x1 = (pred_bbox[0] - pred_bbox[2] / 2) * w
            y1 = (pred_bbox[1] - pred_bbox[3] / 2) * h
            x2 = (pred_bbox[0] + pred_bbox[2] / 2) * w
            y2 = (pred_bbox[1] + pred_bbox[3] / 2) * h
            
            # Convert image from CHW to HWC
            img = test_images[i].permute(1, 2, 0).cpu().numpy()

            # Visualize the image and the predicted bounding box
            fig, ax = plt.subplots(1)
            ax.imshow(img)
            rect = patches.Rectangle((x1, y1), x2 - x1, y2 - y1, linewidth=2, edgecolor='r', facecolor='none')
            ax.add_patch(rect)
            plt.show()  # Display the image with bounding box    

    return output  # Return the output for further use if needed
# Main Execution
if __name__ == "__main__":
    transform = transforms.Compose([transforms.Resize((224, 224)), transforms.ToTensor()])
    image_folder = "../gate_ml/gate_dataset/images/train"
    label_folder = "../gate_ml/gate_dataset/labels/train"
    image_folder_val = "../gate_ml/gate_dataset/images/val"
    label_folder_val = "../gate_ml/gate_dataset/labels/val"
    
    dataset = YoloBBoxDataset(image_folder, label_folder, transform=transform)
    datasetVal = YoloBBoxDataset(image_folder_val, label_folder_val, transform=transform)

    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)
    dataloaderVal = DataLoader(datasetVal, batch_size=32, shuffle=True)
    
    visualize_data(dataloader)
    
    model = DeepConvNet()
    train_model(model, dataloader, epochs = 30, lr = 0.01)
    
    for images, labels in dataloaderVal:
        visualize_inference(model, images, device)
