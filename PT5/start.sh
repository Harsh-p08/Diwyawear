#!/bin/bash

# Set permissions for the LiDAR
sudo chmod a+rw /dev/ttyTHS1
sudo chmod a+rw /dev/gpiochip1
sudo chmod a+rw /dev/gpiochip0

# Run your Python script
sudo /usr/bin/python3 /home/pt5/Downloads/person.py

