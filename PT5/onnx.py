import torch
import torchvision.models as models
from ultralytics import YOLO

# Load your PyTorch model
model = YOLO("yolov8n.pt")
model.eval()

# Define dummy input (change this according to your model's input requirements)
dummy_input = torch.randn(1, 3, 224, 224)

# Export the model to ONNX format
torch.onnx.export(model, dummy_input, "resnet18.onnx", verbose=True)

print("Model exported successfully to 'resnet18.onnx'")

