import matplotlib.pyplot as plt
import matplotlib.patches as patches
from YoloDataset.py import YoloDataset as YD
# Define transformations (resize and convert to tensor)
transform = transforms.Compose([
    transforms.Resize((224, 224)),  # Resize image to (224, 224)
    transforms.ToTensor(),  # Convert to tensor
])

# Set paths for images and labels
image_folder = '../gate_ml/gate_dataset/images'  # Directory containing your image files
label_folder = '../gate_ml/gate_dataset/labels'  # Directory containing your YOLO label files

# Create dataset and DataLoader
dataset = YoloDataset(image_folder=image_folder, label_folder=label_folder, transform=transform)
dataloader = DataLoader(dataset, batch_size=4, shuffle=True)

# Get a batch of data
images, labels = next(iter(dataloader))

# Visualize a few images
for i in range(min(4, len(images))):
    image = images[i].permute(1, 2, 0).numpy()  # Convert tensor to HWC format for plt.imshow
    fig, ax = plt.subplots(1)
    ax.imshow(image)

    # Get the bounding boxes
    for label in labels[i]:
        class_id, x1, y1, x2, y2 = label
        rect = patches.Rectangle((x1, y1), x2 - x1, y2 - y1,
                                 linewidth=2, edgecolor='r', facecolor='none')
        ax.add_patch(rect)

    plt.show()
