#! /bin/bash 

# Source where the simulation files will be
MARA_SIM_PATH=/home/simulations/ros2_sims_ws/install/setup.bash
source $MARA_SIM_PATH
source /usr/share/gazebo/setup.sh
ros2 launch mara_gazebo mara.launch.py
