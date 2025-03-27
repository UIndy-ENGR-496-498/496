#!/bin/bash

colcon build

source install/setup.bash

ros2 run gps_pub gps_node
