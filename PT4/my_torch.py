import torch

# Load the model without loading its weights
model = torch.jit.load("yolov8n.pt", map_location=torch.device('cpu'))

# Get the input and output names
input_names = model._actual_script_module._c.input_names()
output_names = model._actual_script_module._c.output_names()

print("Input names:", input_names)
print("Output names:", output_names)

