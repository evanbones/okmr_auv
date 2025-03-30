import torch
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
import os
import numpy as np

class YoloDataset(Dataset):
    def __init__(self, image_folder, label_folder, transform=None):
        """
        Args:
            image_folder (str): Path to the folder with images.
            label_folder (str): Path to the folder with YOLO-formatted labels.
            transform (callable, optional): Optional transformation to be applied on an image.
        """
        self.image_folder = image_folder
        self.label_folder = label_folder
        self.transform = transform
        self.image_files = os.listdir(image_folder)  # Assuming images are in a common format (like .jpg, .png)
        
    def __len__(self):
        return len(self.image_files)
    
    def __getitem__(self, idx):
        # Get the image path
        img_name = self.image_files[idx]
        img_path = os.path.join(self.image_folder, img_name)
        
        # Load the image
        image = Image.open(img_path).convert("RGB")
        
        # Get the corresponding label path (assuming labels have the same name as the image but with .txt extension)
        label_name = img_name.split('.')[0] + '.txt'
        label_path = os.path.join(self.label_folder, label_name)
        
        # Load the YOLO labels
        boxes = self.load_yolo_labels(label_path, image.size)
        
        # Apply transformations if any
        if self.transform:
            image = self.transform(image)
        
        return image, boxes
    
    def load_yolo_labels(self, label_path, image_size):
        """
        Load the YOLO labels and convert them to the appropriate format.
        YOLO format: <class_id> <x_center> <y_center> <width> <height>
        """
        boxes = []
        with open(label_path, 'r') as file:
            for line in file:
                # Split the label line and get the bounding box info
                label = list(map(float, line.strip().split()))
                class_id = int(label[0])
                x_center, y_center, width, height = label[1:]

                # Denormalize coordinates (convert from [0, 1] range to image coordinates)
                w, h = image_size
                x_center *= w
                y_center *= h
                width *= w
                height *= h

                # Add the bounding box ( x1, y1, x2, y2) NOTE will not be classifying bounding boxes
                boxes.append([ x_center, y_center, width, height])
        
        # Convert boxes to a tensor (or empty tensor if no boxes)
        if len(boxes) == 0:
            return torch.zeros((0, 5))  # No boxes
        else:
            return torch.tensor(boxes, dtype=torch.float32)
