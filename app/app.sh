#!/bin/bash
# source ros and python3 components
source ~/myenv/bin/activate
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
# install gym environment
pip3 install -e /catkin_ws/src/RL-with-3DOF-Robots/fhtw3dof/gym-fhtw3dof
# initiate plaidml
plaidml-setup
# start ros nodes and put them to the background
#roslaunch mairobot MairobotClickAndGo.launch &
#roslaunch saimon SAImon.launch coll_map:=parkour.yaml run_on_real_robot:=false &
roslaunch mairobot Mairobot.launch &
sleep 7
python3