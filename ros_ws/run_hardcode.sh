#!/bin/bash
colcon build
source install/setup.bash
ros2 launch launch_nodes hc_driver.launch
