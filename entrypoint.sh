#!/bin/bash
# Source ros and python3 components
source ~/myenv/bin/activate
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

export TURTLEBOT3_MODEL=burger

# Start jupyterlab
export JUPYTER_ENABLE_LAB=yes
export JUPYTER_TOKEN=docker
jupyter-lab --ip 0.0.0.0 -IdentityProvider.token='ros_ml_container' --no-browser --allow-root &

# Start rosbridge server in the background for foxglove connection
#roslaunch rosbridge_server rosbridge_websocket.launch &

exec "$@"

