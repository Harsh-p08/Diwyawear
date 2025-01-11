import torch 

if torch.cuda.is_available():
    print("TRUE")
else:
    print("FALSE")
