#!/bin/bash

echo "Setting up functions for ROS Noetic + Catkin WS + PX4"

function simsrc {
  ROSWS=~/catkin_ws
  PX4FW=~/src/PX4-Autopilot
  source /opt/ros/noetic/setup.bash
  source $ROSWS/devel/setup.bash
  source $PX4FW/Tools/setup_gazebo.bash $PX4FW $PX4FW/build/px4_sitl_default
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4FW
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4FW/Tools/sitl_gazebo
  echo simsrc
}
export -f simsrc
echo "new command: simsrc"

function simbld {
  simsrc
  cd $ROSWS
  catkin_make
  simsrc
  echo simbld
}
export -f simbld
echo "new command: simbld"
