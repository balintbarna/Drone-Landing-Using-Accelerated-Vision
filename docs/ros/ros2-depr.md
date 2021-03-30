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

Creating a new python package
```
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

Adding new python nodes requires adding them to the *entry_points* in *setup.py*

New C++ package
```
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

It is possible, but complicated, to create mixed packages.

Running a node
```
ros2 run my_package my_node
```
