#!/bin/bash

sed -i "26s/_MAX_FLOAT = .*/_MAX_FLOAT = np.maximum_sctype('f4')/" /usr/lib/python3/dist-packages/transforms3d/quaternions.py && \
sed -i "27s/_FLOAT_EPS = .*/_FLOAT_EPS = np.finfo('f4').eps/" /usr/lib/python3/dist-packages/transforms3d/quaternions.py

echo "Search and replace completed."

# Build the workspace
colcon build 
source install/setup.bash

echo "Build completed."

ros2 run arduino_serial_reader serial_reader_node
