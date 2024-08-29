#!/bin/bash

# Script to copy saved LIDAR bag files saved under REMOTE_PATH (Rasb PI's path) to LOCAL_PATH (your PC)
# USE: open a ssh connection to the rasbpi, launch the ./save.sh to start recording. Ctrl+C to stop and copy files.

# PROBLEM: I think the folders are not correctly mounted

# Set variables
CONTAINER_NAME="unilio_arm"
LAUNCH_FILE="run_without_rviz.launch"
ROS_PACKAGE="unitree_lidar_ros"
LOCAL_USER="sqdr"
LOCAL_HOST="10.42.0.1"  # main pc's ip
LOCAL_PATH="/home/sqdr/Desktop"
PI_PATH="/home/pi_lio/data/" # pi path
OTHER_SCRIPT="./home/unilidar_sdk/unitree_lidar_sdk/bin/example_lidar"

# Function to clean up on exit
cleanup() {
  echo "Script interrupted. Stopping container and copying files..."

  echo "Calling the stop spinning script..."
  # Uncomment if you have a stop spinning script
  # $OTHER_SCRIPT &
  # OTHER_SCRIPT_PID=$!

  docker exec -d $CONTAINER_NAME bash -c "rosnode kill -a"

  # Wait for 1 second and then send SIGINT to the other script
  sleep 1
  # Uncomment if you are sending SIGINT to the other script
  # echo "Sending SIGINT to the other script..."
  # kill -SIGINT $OTHER_SCRIPT_PID
  # wait $OTHER_SCRIPT_PID

  # Stop the Docker container
  echo "Stopping the Docker container..."
  docker stop $CONTAINER_NAME

  # Find the latest files in the local folder
  echo "Finding the latest files..."
  LATEST_FILES=$(find $PI_PATH -type f -printf '%T+ %p\n' | sort -r | head -n 1 | awk '{print $2}')

  # Copy the latest files to the remote computer
  echo "Copying the latest files to the remote computer..."
  for FILE in $LATEST_FILES; do
    scp "$FILE" "$LOCAL_USER@$LOCAL_HOST:$LOCAL_PATH"
  done

  echo "Files copied successfully. Exiting script."
  exit 0
}

# Trap Ctrl+C (SIGINT) to run the cleanup function
trap cleanup SIGINT

# Step 1: Start the Docker container
echo "Starting the Docker container..."
docker start $CONTAINER_NAME

# Ensure the container started successfully
if [ $? -ne 0 ]; then
 echo "Failed to start the Docker container. Exiting."
 exit 1
fi
echo "Docker container started successfully."

# Step 2: Launch the ROS launch file inside the container
echo "Launching the ROS launch file inside the container..."
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && cd /home/validation_ws/ && source devel/setup.bash && roslaunch $ROS_PACKAGE $LAUNCH_FILE"
if [ $? -ne 0 ]; then
  echo "Failed to launch ROS launch file. Exiting."
  exit 1
fi
echo "ROS launch file started"

sleep 1

#echo "ROS mapping node started"
#docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && cd /home/validation_ws/ && source devel/setup.bash && roslaunch point_lio_unilidar mapping_unilidar.launch"
#if [ $? -ne 0 ]; then
#  echo "Failed to start ROS mapping node. Exiting."
#  exit 1
#fi

#sleep 1

echo "ROS static transform published"
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && cd /home/validation_ws/ && source devel/setup.bash && rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 unilidar_lidar camera_init 100"
if [ $? -ne 0 ]; then
  echo "Failed to publish ROS static transform. Exiting."
  exit 1
fi

sleep 1

docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && cd /home/validation_ws/ && source devel/setup.bash && rosbag record -a -o $PI_PATH"
if [ $? -ne 0 ]; then
  echo "Failed to start ROSBAG recording. Exiting."
  exit 1
fi
echo "ROSBAG recording started. Press Ctrl+C to stop and copy files."

# Wait indefinitely until the script is interrupted
while true; do
  sleep 1
done

