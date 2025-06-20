#!/bin/bash
cd ~/drone/drone_ws/
source install/setup.bash
ros2 run camera camera_streamer
