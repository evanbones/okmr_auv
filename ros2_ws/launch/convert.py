import torch
import torchvision.models as models

# Load your model
model = torch.load('LidModelRGB.pt')
model.eval()

# Example dummy input (adjust shape as needed)
dummy_input = torch.randn(1, 3, 640, 640)

# Export to ONNX
torch.onnx.export(model, dummy_input, "LidModelRGB.onnx", opset_version=12)

