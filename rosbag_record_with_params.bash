#!/bin/bash

# Ensure the ROS environment is sourced
source /opt/ros/noetic/setup.bash  # Change to your ROS distro if different

# Get timestamp
TIMESTAMP=$(date +"%Y-%m-%d-%H-%M-%S")

# Move to home
cd .
mkdir ~/bags
cd ~/bags

# Create folder with timestamp
mkdir ./$TIMESTAMP
cd $TIMESTAMP

# Filenames with Timestamps
BAG_FILE="./$TIMESTAMP.bag"
PARAMS_FILE="./$TIMESTAMP.yaml"

# Function to dump parameters periodically
dump_params() {
    rosparam dump $PARAMS_FILE
}

dump_params

# Get all active topics and save them into a variable
TOPICS=$(rostopic list)

# Start recording all topics with rosbag in the background
echo "Starting rosbag record for all topics..."
rosbag record $TOPICS -O $BAG_FILE &

# Get the PID of the background rosbag process to kill later
BAG_PID=$!

# Periodically dump parameters while rosbag is recording.
# while kill -0 $BAG_PID 2>/dev/null; do
#     dump_params
#     sleep 10  # 10s
# done
wait $BAG_PID

echo "Recording complete. Bag file saved as $BAG_FILE and parameters saved to $PARAMS_FILE."