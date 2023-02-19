#!/bin/bash
# source ros and python3 components
source ~/myenv/bin/activate
source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/local_setup.bash

#export TURTLEBOT3_MODEL=burger

export JUPYTER_ENABLE_LAB=yes
export JUPYTER_TOKEN=docker
jupyter-lab --ip 0.0.0.0 -IdentityProvider.token='ros_ml_container' --no-browser --allow-root &

exec "$@"

