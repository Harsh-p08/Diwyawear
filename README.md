# Object Detection and Distance Measurement with YOLOv8, RealSense, and LiDAR

This project utilizes the YOLOv8 object detection model, Intel RealSense camera, and optional LiDAR sensor to detect objects, measure their distances, and control GPIO pins accordingly. It supports two versions (protoypes):

- **PV4**: Uses only the RealSense camera for distance measurement.
- **PV5**: Integrates LiDAR for enhanced distance measurement and safety features.

## Features

- **Object Detection**: Detect objects in real-time using the YOLOv8 model.
- **Distance Measurement**: 
  - PV4: Uses RealSense depth data to measure distances.
  - PV5: Incorporates LiDAR for precise distance measurement.
- **GPIO Control**: Automatically toggles GPIO pins based on detected objects and their distances.
- **Visual Feedback**: Displays bounding boxes, object labels, and distances on video frames.

## Requirements

### Hardware
- Intel RealSense Camera
- Jetson Nano (or compatible system with GPIO support)
- LiDAR Sensor (for PV5 only)

### Software and Libraries
- Python 3.10
- OpenCV
- Intel RealSense SDK
- PyTorch
- Ultralytics YOLOv8
- GPIO Library (e.g., RPi.GPIO or gpiozero)
- Serial Communication Library (e.g., pyserial for LiDAR)

## Installation

1. **Clone the Repository**:
    ```bash
    git clone https://github.com/Harsh-p08/Diwyawear
    cd PT5 or cd PT4
    ```

2. **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3. **Setup YOLOv8**:
    - Follow the [Ultralytics YOLOv8 documentation](https://docs.ultralytics.com) to install the YOLO model and weights.

4. **Install Intel RealSense SDK**:
    - Refer to the [Intel RealSense installation guide](https://github.com/IntelRealSense/librealsense).

5. **Configure GPIO**:
    - Ensure your GPIO pins are correctly set up as per your hardware configuration.

## Usage

### Run PV4 (RealSense Only)
```bash
cd PT4
python3 person.py
```

### Run PV5 (RealSense + LiDAR)
```bash
cd PT5
python3 person.py
```

### Controls
- Press **'q'** to exit the application.

## Code Overview

### Common Features (PV4 and PV5)
1. **Library Imports**: Includes OpenCV, RealSense SDK, YOLOv8, GPIO, and time management libraries.
2. **YOLO Model**: Loaded for real-time object detection.
3. **GPIO Setup**: Defines output pins and initializes them to LOW.
4. **Camera Calibration**: Sets camera matrix and distortion coefficients.
5. **Object Detection Loop**:
   - Captures frames from RealSense.
   - Detects objects using YOLOv8.
   - Measures object distances and identifies the nearest object.
   - Toggles GPIO pins based on detected object and distance.
   - Displays processed frames with bounding boxes and labels.

### PV5-Specific Features
- **LiDAR Integration**: 
  - Initializes serial communication with the LiDAR sensor.
  - Uses LiDAR data for precise distance measurement.
  - Overrides RealSense distance if LiDAR reports a closer object.
- **Safety Logic**:
  - If LiDAR detects an object within 0.5 meters (15 units), sets all GPIO pins to HIGH.

## Example Outputs

- **PV4**:
  - Displays detected objects and distances using RealSense.

- **PV5**:
  - Combines RealSense and LiDAR for improved accuracy.
  - Activates safety features when objects are too close.

## Troubleshooting

- **YOLO Not Detecting Objects**:
  - Ensure the YOLOv8 weights are correctly downloaded and loaded.
  - Test with different confidence thresholds.

- **RealSense Camera Issues**:
  - Verify that the Intel RealSense SDK is correctly installed.
  - Check camera connections.

- **LiDAR Not Responding (PV5)**:
  - Ensure serial communication is properly set up.
  - Test LiDAR with standalone tools to verify functionality.

## Contribution

Contributions are welcome! Feel free to fork the repository and submit a pull request.


Developed in a Research Project by [Harsh Patel](https://github.com/Harsh-p08)
