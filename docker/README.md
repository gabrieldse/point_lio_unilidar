# How to use the docker container for data capture:


1. Build the docker image using:
```bash
docker build .
```
2. Run the script to download repositories and setup data folder
```bash
./ros_package_install.sh
```
 
Or just download the docker built image:
```bash
docker pull gabrieldse/unilio_arm:latest
```

Commands:

docker start <image_name> & docker exec -it <image_name> /bin/bash

Script to run the data recording (ROSBAG Recordings) and transfer them via ssh:

```bash
./save.sh
```
You should change the following parameters at the begging of the file in order for it to work.

- CONTAINER_NAME="unilio_arm"
- LAUNCH_FILE="mapping_unilidar_record.launch"
- ROS_PACKAGE="point_lio_unilidar"
- REMOTE_USER="pi_lio"
- REMOTE_HOST="=192.168.43.131"
- REMOTE_PATH="/home/raw_lidar_data"
- LOCAL_PATH="/home/Desktop"