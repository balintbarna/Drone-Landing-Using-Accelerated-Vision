#!/bin/bash

echo "Setting up functions for live deployment"

function livesrc {
  ROSWS=~/catkin_ws
  source /opt/ros/melodic/setup.bash
  source $ROSWS/devel/setup.bash
  echo livesrc
}
export -f livesrc
echo "new command: livesrc"

function bindnb {
  ROSWS=~/catkin_ws
  REPO_NAME=Drone-Landing-Using-Accelerated-Vision
  REPO_PATH=$ROSWS/src/$REPO_NAME
  mkdir -p $PYNQ_JUPYTER_NOTEBOOKS/dai_jupyter
  sudo mount --bind $REPO_PATH/dai_jupyter/ $PYNQ_JUPYTER_NOTEBOOKS/dai_jupyter/
  echo "notebook binding done"
}
export -f bindnb
echo "new command: bindnb"
