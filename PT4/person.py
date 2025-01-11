import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import Jetson.GPIO as GPIO
import serial
import time

iter = 0

# Load the YOLOv5 model
model = YOLO("yolov8n.pt")

# Set the reference distance (in meters)
ref_distance = 5.0

# Define the output pins for GPIO control
output_pins = [18,19,21,22,23,24,26,31,11,12]

# Define lists of stationary and semi-stationary objects
Stationary_Objects = ['dining table','bench','couch','traffic light','tv','laptop','refrigerator','book']
Semi_Stationary_Objects = ['person','umbrella', 'bicycle', 'motorcycle','car','bus','dog','cat','cow']

# Initialize a serial connection on port '/dev/ttyTHS1' with baud rate 115200
ser = serial.Serial("/dev/ttyTHS1", 115200)

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
for pin in output_pins:
    GPIO.setup(pin, GPIO.OUT,  initial=GPIO.LOW)

# Set the camera matrix and distortion coefficients
camera_matrix = np.array([[379.45187378, 0, 324.09848022], [0, 379.45187378, 238.02722168], [0, 0, 1]])
distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# Set the object size (in meters) at the reference distance
obj_size = 1.0

# Set the video capture 
cap = cv2.VideoCapture(1)

# Start the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Initialize the depth scale
depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

while True:
    # Read the frame
    ret, frame = cap.read()
    (H, W) = frame.shape[:2]

    # Perform object detection using YOLO
    results = model(frame)

    # Draw the bounding boxes and labels
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        continue

    nearest_object = None
    min_dist = float('inf')

    # Calculating distance from lidar
    count = ser.in_waiting
    if count > 8:
            recv = ser.read(9)  
            ser.reset_input_buffer()  
            
            if recv[0] == 0x59 and recv[1] == 0x59:     #python3
                distance_l = recv[2] + recv[3] * 256
                strength = recv[4] + recv[5] * 256
                ser.reset_input_buffer()

    iter = iter + 1

    # Iterate over detected objects
    for result in results:
        print(iter, "> ", result)      
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = model.names[int(box.cls)]

            # Calculate the distance
            height = y2 - y1
            distance_mm = depth_frame.get_distance(x1 + (x2 - x1) // 2, y1 + (y2 - y1) // 2) * 1000  # Convert to millimeters
            dist = distance_mm * depth_scale  # Convert to meters

            # Update nearest object and distance if closer than current nearest
            if dist < min_dist:
                min_dist = dist 
                nearest_object = label

            # Reset all GPIO pins to LOW
            for pin in output_pins:
                GPIO.output(pin, GPIO.LOW)
          
            # Set specific GPIO pin based on the nearest object
            if (nearest_object == 'person' or nearest_object == 'umbrella'): #18
                GPIO.output(output_pins[0], GPIO.HIGH)
            elif (nearest_object == 'bicycle' or nearest_object == 'motorcycle'): #19
                GPIO.output(output_pins[1], GPIO.HIGH)
            elif (nearest_object == 'dining table' or nearest_object == 'couch' or nearest_object == 'bench'): #21
                GPIO.output(output_pins[2], GPIO.HIGH)
            elif (nearest_object == 'traffic light'): #22
                GPIO.output(output_pins[3], GPIO.HIGH)
            elif (nearest_object == 'book'): #23
                GPIO.output(output_pins[4], GPIO.HIGH)
            elif (nearest_object == 'dog' or nearest_object == 'cat' or nearest_object == 'cow'): #24
                GPIO.output(output_pins[5], GPIO.HIGH)
            elif (nearest_object == 'car'): #26
                GPIO.output(output_pins[6], GPIO.HIGH)
            elif (nearest_object == 'bus'): #31
                GPIO.output(output_pins[7], GPIO.HIGH)
            elif (nearest_object == 'laptop' or nearest_object == 'tv'): #11
                GPIO.output(output_pins[8], GPIO.HIGH)
            elif (nearest_object == 'refrigerator'): #12
                GPIO.output(output_pins[9], GPIO.HIGH)

#            if(distance_l<15):
 #               for pin in output_pins:
  #                  GPIO.output(pin, GPIO.HIGH) 


            # Draw the object label and distance on the frame
            cv2.putText(frame, f"{label} {dist:.2f} m", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Draw the bounding box color based on distance
            if (dist <= 0.5):
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            elif(0.5<dist<=1.5):
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            elif (dist>1.5): 
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
  
    # Print the nearest object and its distance
    print(f"Nearest : {nearest_object}, Distance: {min_dist:.2f} meters.")

    # Display the frame
    cv2.imshow("Object Detection and Distance Estimation", frame)

    # Exit if the user presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and destroy all windows
cap.release()
cv2.destroyAllWindows()
