import cv2
import numpy as np
import torch

# Load the ONNX model
onnx_model = torch.onnx.load("yolov8.onnx")

# Set the input name and shape
onnx_input_name = "input"
onnx_input_shape = (1, 3, 416, 416)

# Create a dummy input tensor with the same shape and data type as the actual input
onnx_dummy_input = torch.randn(*onnx_input_shape)

# Create an ONNX session
onnx_session = torch.onnx.TracingSession(onnx_model, input_names=[onnx_input_name], output_names=['output'])

# Set the camera matrix and distortion coefficients
camera_matrix = np.array([[379.45187378, 0, 324.09848022], [0, 379.45187378, 238.02722168], [0, 0, 1]])
distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Set the video capture (replace 0 with your camera index if needed)
cap = cv2.VideoCapture(1)

while True:
    # Read the frame
    ret, frame = cap.read()

    # Perform object detection
    onnx_outputs = onnx_session.run(None, {onnx_input_name: onnx_dummy_input.to(device).cpu().numpy()})
    onnx_outputs = [torch.from_numpy(onnx_output) for onnx_output in onnx_outputs]
    onnx_outputs = [onnx_output.to(device) for onnx_output in onnx_outputs]
    results = model.postprocess(onnx_outputs)

    # Draw the bounding boxes and labels
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = model.names[int(box.cls)]

            # Calculate the distance
            height = y2 - y1

            distance_mm = frame.get_distance(x1 + (x2 - x1) // 2, y1 + (y2 - y1) // 2) * 1000  # Convert to millimeters
            dist = distance_mm * depth_scale  # Convert to meters

            # Draw the distance on the rectangle
            cv2.putText(frame, f"{label} {dist:.2f} m", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Draw the bounding box
            if (dist <= 0.5):
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            elif(0.5<dist<=1.5):
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            elif (dist>1.5): 
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Object Detection", frame)

    # Exit if the user presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and destroy all windows
cap.release()
cv2.destroyAllWindows()
