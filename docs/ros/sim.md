1. Have [Ubuntu Focal](http://www.releases.ubuntu.com/20.04/)
1. Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (should also work with Melodic with python2)
1. Install MAVROS Extras (`apt install ros-noetic-mavros-extras`) and its [GeographicsLib dataset](https://docs.px4.io/master/en/ros/mavros_installation.html)
1. Clone [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) with submodules: `git clone --recursive https://github.com/PX4/PX4-Autopilot.git`
1. Refer to [this page](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html) for installing dependencies. In short, run `bash ./Tools/setup/ubuntu.sh` in the folder of the cloned Autopilot.
1. [Build](https://docs.px4.io/master/en/simulation/ros_interface.html) the autopilot for Gazebo simulation: `DONT_RUN=1 make px4_sitl_default gazebo`
1. Make sure that `python` references `python3`. Use `update-alternatives --config python` to select the correct version. If v3 cannot be selected, use `update-alternatives --install /usr/bin/python python /usr/bin/python3 1` to add it.
1. Install `opencv-python` with pip.
1. Create a Catkin workspace (`~/catkin_ws/src`)
1. Make sure that in `sim.bash` the `ROSWS` and `PX4FW` variables are pointing to the right path
1. Source `sim.bash` on every terminal session used for the simulation
1. `sim.bash` registers a command to source relevant paths, like the path to the ROS Noetic distribution, the setup script of the catkin workspace, and the PX4 Autopilot related setup scripts. It also registers a command to build the catkin workspace, re-source it afterwards. To run the simulation the sourcing command needs to run first in each related terminal session.
1. Call `roslaunch dai_sim sim.launch`
1. Run Gazebo GUI with the `gzclient` command in a terminal.
