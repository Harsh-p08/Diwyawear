#!/bin/bash

# Set permissions for the LiDAR
sudo chmod a+rw /dev/ttyTHS1

# Run your Python script
/usr/bin/python3 /home/dw/Downloads/person.py

