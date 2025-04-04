#!/bin/bash

# Set the ROS workspace
source ../install/setup.bash

# Set the output directory for the ros bag
output_directory="./"

# Create a unique bag file name with a timestamp
bag_filename="odom_bag_$(date +"%Y%m%d_%H%M%S").bag"

# Set the ROS topic you want to record
topic="/odom"

# Record the ROS bag
ros2 bag record -o "$output_directory/$bag_filename" "$topic"
