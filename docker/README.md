# How to use the docker container for data capture:

1. Build the docker image using:
```bash
cd point_lio_unilidar/docker
docker build -t unilio_arm .
```
2. Run the script to download repositories and build packages:

It is recommended to have a minimum of 1GB of swap for the builds.
```bash
./create_container.sh
```
 
Or just download the docker built image (currently only for AArch64. So no Raspian Buster):
```bash
docker pull gabrieldse/unilio_arm:latest
```

To check if the container is working as expected, during a capture, on a new terminal inside the container:

```bash
rostopic echo /unilidar/cloud
```
to see the point cloud array on the terminal.

# Transfer the Point Clouds:

Script to run the data recording (ROSBAG Recordings) and transfer them via ssh:

```bash
./save.sh
```
You should change the following parameters at the begging of the file in order for it to work.

- LOCAL_USER="sqdr" # main pc's user
- LOCAL_HOST="192.168.43.158"  # main pc's ip
- LOCAL_PATH="/home/sqdr/Desktop" # path to save the bagfiles
- PI_PATH="/home/pi_lio/data/" # pi's path

### Commands:

docker start <image_name> & docker exec -it <image_name> /bin/bash

