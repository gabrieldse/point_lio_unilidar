#!/bin/bash

# Script to copy saved LIDAR bag files saved under REMOTE_PATH (Rasb PI's path) to LOCAL_PATH (your PC)
# USE: open a ssh connection to the rasbpi, launch the ./save.sh to start recording. Ctrl+C to stop and copy files.

# Personalize this part:
LOCAL_USER="sqdr" # main pc's user
LOCAL_HOST="192.168.43.158"  # main pc's ip
LOCAL_PATH="/home/sqdr/Desktop" # path to save the bagfiles
PI_PATH="/home/pi_lio/data/" # pi's path

# Application specific
CONTAINER_NAME="unilio_arm"
LAUNCH_FILE="run_without_rviz.launch"
ROS_PACKAGE="unitree_lidar_ros"

# Function to clean up on exit
cleanup() {
  echo "[save script] Script interrupted. Stopping container and copying files..."
  echo "[save script] Calling the stop spinning script..."
  # TODO - command to stop the lidar motors

  docker exec -d $CONTAINER_NAME bash -c "rosnode kill -a"
  sleep 2

  echo "[save script] Stopping the Docker container..."
  docker stop $CONTAINER_NAME

  echo "[save script] Finding the latest file..."
  LATEST_FILES=$(find $PI_PATH -type f -printf '%T+ %p\n' | sort -r | head -n 1 | awk '{print $2}')

  echo "[save script] Copying the latest files to the remote computer..."
  for FILE in $LATEST_FILES; do
    scp "$FILE" "$LOCAL_USER@$LOCAL_HOST:$LOCAL_PATH"
  done

  echo "[save script] Files copied successfully. Exiting script."
  exit 0
}

trap cleanup SIGINT

echo "[save script] Starting the Docker container..."
docker start $CONTAINER_NAME

if [ $? -ne 0 ]; then
 echo "[error] Failed to start the Docker container. Exiting."
 exit 1
fi
echo "[save script] Docker container started successfully."

echo "[save script] Launching the ROS launch file inside the container..."
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && source unilidar_sdk/unitree_lidar_ros/devel/setup.bash && roslaunch $ROS_PACKAGE $LAUNCH_FILE"
if [ $? -ne 0 ]; then
  echo "[error] Failed to launch ROS launch file. Exiting."
  exit 1
fi
echo "[save script] ROS launch file started"

sleep 2

#echo "[save script] ROS mapping node started"
#docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && source /home/validation_ws/devel/setup.bash && roslaunch point_lio_unilidar mapping_unilidar.launch"
#if [ $? -ne 0 ]; then
#  echo "[error] Failed to start ROS mapping node. Exiting."
#  exit 1
#fi
#sleep 1

docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 unilidar_lidar camera_init 230"
if [ $? -ne 0 ]; then
  echo "[error] Failed to publish ROS static transform. Exiting."
  exit 1
fi
echo "[save script] ROS static transform camera <-> lidar being published."

sleep 1

docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && rosbag record -a -o $PI_PATH"
if [ $? -ne 0 ]; then
  echo "[error] Failed to start ROSBAG recording. Exiting."
  exit 1
fi
echo "[save script] ROSBAG recording started. Press Ctrl+C to stop and copy files."

while true; do
  sleep 1
done
