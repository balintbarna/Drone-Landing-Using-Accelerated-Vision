After going through the difficulties of installing *ros-dashing* and the PX4 message bridge on the board, you will have to setup a few more things.

Create your workspace, the default convention is *~/dev_ws/src*

Then add this to your *.bashrc*
```
function dronesetup {
  ROSWS=~/dev_ws
  REPO=Drone-Landing-Using-Accelerated-Vision
  source $ROSWS/src/$REPO/setup.bash
  echo dronesetup
}
export -f dronesetup

function dronebuild {
  dronesetup
  cd $ROSWS
  colcon build
  dronesetup
  echo dronebuild
}
export -f dronebuild
```
