import torch
import torchvision
from ultralytics import YOLO

# Load the PyTorch model
model = YOLO("yolov8n.pt")

# Create a dummy input tensor
dummy_input = torch.randn(10, 3, 640, 640, device ="cuda")

# Export the model to ONNX format
torch.onnx.export(model,               # model being run
                  dummy_input,         # model input (or a tuple for multiple inputs)
                  "yolov8n.onnx",        # where to save the model (can be a file or file-like object)
                  export_params=True,  # store the trained parameter weights inside the model file
                  opset_version=12,    # the ONNX version to export the model to
                  do_constant_folding=True,  # whether to execute constant folding for optimization
                  input_names = ['input'],   # the model's input names
                  output_names = ['output'], # the model's output names
                  dynamic_axes={'input' : {0 : 'batch_size'},    # variable length axes
                                'output' : {0 : 'batch_size'}})
