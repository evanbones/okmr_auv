import torch
import torch.nn as nn
import torch.optim as optim

# Create a super basic dataset
class SimpleCNN(nn.Module):
    def __init__(self):
        super(SimpleCNN, self).__init__()
        
        # Convolutional layers
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=1)  # 3 input channels (RGB), 32 output channels
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)  # 32 input channels, 64 output channels
        
        # Pooling layer
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        
        # Fully connected layer
        self.fc1 = nn.Linear(64 * 64 * 64, 512)  # Assuming input image size of 256x256, after pooling it's 64x64
        self.fc2 = nn.Linear(512, 4)  # Output 4 values for bounding box: [x_min, y_min, x_max, y_max]

    def forward(self, x):
        # Apply convolutional layers with ReLU activation
        x = self.pool(torch.relu(self.conv1(x)))
        x = self.pool(torch.relu(self.conv2(x)))
        
        # Flatten the feature map for the fully connected layer
        x = x.view(-1, 64 * 64 * 64)  # Flatten the output
        
        # Fully connected layers
        x = torch.relu(self.fc1(x))
        x = self.fc2(x)  # Final output layer
        
        return x  # Output bounding box coordinates [x_min, y_min, x_max, y_max]

model = SimpleCNN()
criterion = nn.MSELoss()  # Mean Squared Error for bounding box prediction
optimizer = optim.Adam(model.parameters(), lr=0.001)

sample_input = torch.randn(1, 3, 256, 256)

output = model(sample_input)
print("Bounding Box Output:", output)
