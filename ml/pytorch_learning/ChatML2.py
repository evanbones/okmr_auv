import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from torch.utils.data import Dataset, DataLoader
from PIL import Image
import os
import torch.cuda.amp as amp
import torchvision.models as models

# Device

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)

# Dataset

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

        w_img, h_img = image.size

        # Convert bbox to absolute coordinates
        bbox_abs = bbox.clone()
        bbox_abs[0] *= w_img
        bbox_abs[1] *= h_img
        bbox_abs[2] *= w_img
        bbox_abs[3] *= h_img

        # Horizontal flip
        if torch.rand(1).item() < 0.5:
            image = transforms.functional.hflip(image)
            bbox_abs[0] = w_img - bbox_abs[0]

        # Resize image
        image = transforms.Resize((224, 224))(image)
        bbox_abs[0] *= 224 / w_img
        bbox_abs[1] *= 224 / h_img
        bbox_abs[2] *= 224 / w_img
        bbox_abs[3] *= 224 / h_img

        # Normalize bbox coordinates
        bbox[0] = bbox_abs[0] / 224
        bbox[1] = bbox_abs[1] / 224
        bbox[2] = bbox_abs[2] / 224
        bbox[3] = bbox_abs[3] / 224

        if self.transform:
            image = self.transform(image)

        return image, bbox

    def load_yolo_bbox(self, label_path):
        with open(label_path, 'r') as file:
            first_line = file.readline().strip().split()
            bbox = torch.tensor(list(map(float, first_line[1:])), dtype=torch.float32)
        return bbox

# Model

class ResNetBBox(nn.Module):
    def __init__(self, pretrained=True):
        super(ResNetBBox, self).__init__()
        self.resnet = models.resnet50(pretrained=pretrained)
        num_features = self.resnet.fc.in_features
        self.resnet.fc = nn.Linear(num_features, 4)

    def forward(self, x):
        return self.resnet(x)

# Visualization

def visualize_bbox(img_tensor, bbox):
    img = img_tensor.permute(1, 2, 0).cpu().numpy()
    h, w, _ = img.shape
    x1 = (bbox[0] - bbox[2] / 2) * w
    y1 = (bbox[1] - bbox[3] / 2) * h
    rect_w = bbox[2] * w
    rect_h = bbox[3] * h

    fig, ax = plt.subplots()
    ax.imshow(img)
    rect = patches.Rectangle((x1, y1), rect_w, rect_h, linewidth=2, edgecolor='r', facecolor='none')
    ax.add_patch(rect)
    plt.show()

def visualize_data(dataloader):
    images, boxes = next(iter(dataloader))
    for i in range(len(images)):
        visualize_bbox(images[i], boxes[i])

# Training

def train_model(model, dataloader, dataloader_val, epochs=10, lr=0.001):
    model = model.to(device)
    criterion = nn.SmoothL1Loss()
    optimizer = optim.Adam(model.parameters(), lr=lr)
    scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=10, gamma=0.1)
    scaler = amp.GradScaler()

    best_loss = float('inf')  # Initialize best loss to a very large value
    best_model_path = "best.pt"

    for epoch in range(epochs):
        model.train()
        running_loss = 0.0
        for images, labels in dataloader:
            images, labels = images.to(device), labels.to(device)
            optimizer.zero_grad()

            with amp.autocast():
                outputs = model(images)
                loss = criterion(outputs, labels)

            scaler.scale(loss).backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=2.0)
            scaler.step(optimizer)
            scaler.update()

            running_loss += loss.item()

        scheduler.step()
        avg_train_loss = running_loss / len(dataloader)
        print(f"Epoch {epoch+1}, Training Loss: {avg_train_loss:.4f}")

        # Validate the model
        val_loss = 0.0
        model.eval()
        with torch.no_grad():
            for images, labels in dataloader_val:
                images, labels = images.to(device), labels.to(device)
                outputs = model(images)
                loss = criterion(outputs, labels)
                val_loss += loss.item()

        avg_val_loss = val_loss / len(dataloader_val)
        print(f"Epoch {epoch+1}, Validation Loss: {avg_val_loss:.4f}")

        # Save the model if it has the best validation loss
        if avg_val_loss < best_loss:
            best_loss = avg_val_loss
            torch.save(model.state_dict(), best_model_path)
            print(f"New best model saved with Validation Loss: {best_loss:.4f}")

    print(f"Training complete. Best model saved as {best_model_path}")

# Inference Visualization

def visualize_inference(model, images, device):
    model.eval()
    images = images.to(device)
    with torch.no_grad():
        preds = model(images).cpu()

    for img_tensor, pred_bbox in zip(images.cpu(), preds):
        visualize_bbox(img_tensor, pred_bbox)

# Main execution

if __name__ == "__main__":
    transform = transforms.Compose([
        transforms.ToTensor()
    ])

    image_folder = "../gate_ml/gate_dataset/images/train"
    label_folder = "../gate_ml/gate_dataset/labels/train"
    image_folder_val = "../gate_ml/gate_dataset/images/val"
    label_folder_val = "../gate_ml/gate_dataset/labels/val"

    dataset = YoloBBoxDataset(image_folder, label_folder, transform=transform)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)
    dataset_val = YoloBBoxDataset(image_folder_val, label_folder_val, transform=transform)
    dataloader_val = DataLoader(dataset_val, batch_size=32, shuffle=False)

    # visualize_data(dataloader)

    model = ResNetBBox(pretrained=True)
    train_model(model, dataloader, dataloader_val, epochs=30, lr=0.01)

    for images, _ in dataloader_val:
        visualize_inference(model, images, device)
        break
