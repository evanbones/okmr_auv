import torch
import torch.nn as nn
import torch.optim as optim

# Create a super basic dataset
X = torch.tensor([[1], [2], [3], [4], [5]], dtype=torch.float32)  # Inputs
Y = torch.tensor([[2, 3], [4, 6], [6, 9], [8, 12], [10, 15]], dtype=torch.float32)  # Bounding boxes

class SimpleModel(nn.Module):
    def __init__(self):
        super(SimpleModel, self).__init__()
        self.fc = nn.Linear(1, 2)  # 1 input, 2 outputs (bounding box coords)

    def forward(self, x):
        return self.fc(x)  # Just a single layer

model = SimpleModel()
criterion = nn.MSELoss()  # Mean Squared Error for bounding box prediction
optimizer = optim.Adam(model.parameters(), lr=0.01)

for epoch in range(1000):
    optimizer.zero_grad()  # Reset gradients
    predictions = model(X)  # Forward pass
    loss = criterion(predictions, Y)  # Compute loss
    loss.backward()  # Backpropagation
    optimizer.step()  # Update weights

    if (epoch + 1) % 10 == 0:
        print(f"Epoch {epoch+1}, Loss: {loss.item():.4f}")

print("Training complete!")

test_input = torch.tensor([[20]], dtype=torch.float32)  # Predict for input=6
test_output = model(test_input)
print("Predicted bounding box:", test_output.detach().numpy())
