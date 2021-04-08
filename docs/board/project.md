1. Have the board running a PYNQ image
1. Refer to the [board setup manual](setup.md) for help
1. Add Ubuntu updates repository
1. Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
1. Install [MAVROS](https://docs.px4.io/master/en/ros/mavros_installation.html) with dataset
1. Install git-lfs, which is used to store AI models, with `sudo apt install git-lfs`
1. Clone repository to `~/catkin_ws/src`
1. Install DPU PYNQ following the instructions in their [README](https://github.com/Xilinx/DPU-PYNQ)
1. Install pyzmq and simple-pid with pip2. PyZMQ may need sudo or a lot of patience to install.
    ```bash
    sudo apt-get install -y python-pip python-requests
    pip2 install simple-pid pyzmq
    ```
1. For user access to camera and USB
    ```bash
    sudo adduser xilinx video
    sudo adduser xilinx dialout
    # reboot after
    ```
    The usergroup name can be checked with 
    ```bash
    # have the camera and PX4 already connected to get reasonable results
    ls -ltrh /dev/video*
    ll /dev/ttyACM0
    ```
1. Source the `live.bash` script
1. Use the `bindnb` command to add the notebooks to the running Jupyter server
1. Use the `livesrc` command to source the ROS env
