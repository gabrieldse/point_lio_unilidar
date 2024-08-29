#!/bin/bash

#Script that sets some configurations for the container.
#OBS: The script should be run only once when creating a container from the image.

#To launch the container do: "docker exec -it <name_of_the_container> /bin/bash"

#To open in an interactive bash interpreter do: "docker run -it <name_of_the_image> /bin/bash".

 

#Troubleshooting:

#If the pi installation is server there is no X server:

#xhost +local:docker # do it locally
#export DISPLAY=:1.0 

# Run the Docker container and keep it running
docker run -d --net=host  \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  -e DISPLAY=$DISPLAY \
  --name unilio_arm \
  -v $HOME/.Xauthority:/root/.Xauthority:ro \
  -v /dev:/dev \
  -e "USER_ID=$(id -u)" -e "GROUP_ID=$(id -g)" \
  --device-cgroup-rule "c 81:* rmw" \
  --device-cgroup-rule "c 189:* rmw" \
  -v /home/pi_lio/data:/home/pi_lio/data \
  --entrypoint /bin/bash unilio_arm:latest -c "tail -f /dev/null"
  

# Clone and install unilidar sdk ros1
docker exec -it unilio_arm /bin/bash -c "
        git clone https://github.com/gabrieldse/unilidar_sdk.git &&
        cd unilidar_sdk/unitree_lidar_ros &&
        source /opt/ros/noetic/setup.bash &&
  
  export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/opt/ros/noetic &&
        catkin_make -j2 -l2"

# Clone and install point_lio_unilidar
docker exec -it unilio_arm /bin/bash -c "
        mkdir -p validation_ws/src && cd validation_ws/src &&
        git clone https://github.com/gabrieldse/point_lio_unilidar.git &&
        cd .. &&
  source /opt/ros/noetic/setup.bash &&
        catkin_make -j2 -l2"

# Create folder to store RAW lidar data
docker exec -it unilio_arm /bin/bash -c "
        mkdir -p pi_lio/data"
