# Flashing

The very first step is to flash the SD card with the base image you want to use. This could be anything from <https://github.com/Avnet/Ultra96-PYNQ> or a custom image you have. Flash the SD card with Rufus, Etcher, or similar.

Insert the SD card, connect the power supply, and press the blue button close to the full size USB next to the micro-USB. This will boot the board.

If you're setting up the board with a display and keyboard + mouse, the next part can be skipped. Just use chromium for the notebooks and bash for terminal. Many mini-DP to HDMI converters don't work with this board, but a simple mini-DP to DP cable worked fine.

# First connection

If you have a USB-Ethernet dongle and router connected to the internet, that's the easiest way to set things up. In that case, connecting through microUSB and the Wifi notebook is not necessary, and you can skip ahead. Otherwise, connect a micro-USB cable to the board, and connect the other end to your PC. The board will be reachable on the network as *pynq*. Open the address in the browser to connect to the Jupyter notebooks.

You can also connect to the board with ```ssh xilinx@pynq```.

The password is *xilinx* for both the notebooks and ssh.

# First wifi connection

You need an internet connection to install things and there is no Ethernet. Set up wifi with the common/wifi notebook in Jupyter.
Modify the end of the notebook, so that it waits for an input before closing the port.
This way you can explicitly close the wifi connection later easily.
```
_ = input("Press enter to close connection")
print("Disconnecting...")
port.reset()
print("Disconnected.")
```

# Get comfortable in the terminal

If you open bash on the board GUI, or open a terminal from Jupyter, you're logged in as root. Log in as user *xilinx*:
```
su -l xilinx
```
*su* is switch user. *-l* loads a new terminal session with the user's environment.

To switch back to root:
```
sudo su
```

Using ssh is also a good option to get a terminal interface.

After getting internet, the next step is to update packages. I could only run this as *root*, not as user *xilinx*. It is advised that you only do anything as root when you must.
```
sudo apt update && apt upgrade -y
```

If you are using the board GUI or your own Linux GUI, terminator is useful. The default terminal on the board is especially bad.
```
sudo apt install -y terminator
```

Get mc for file browser view over terminal.
```
sudo apt install -y mc
```

If you're using SSH from Windows, Terminus is a nice terminal with split view support and a lot more.
Install with Scoop, or just download from their GitHub repo.
Also get WinSCP to manage files over SSH.
```
scoop install terminus winscp
```

Add the fingerprint of your SSH client from PowerShell for passwordless login. If you haven't yet, you need to generate your ssh keys with *ssh-keygen*, and create the folder structure on the target first. From linux, use ssh-copy-id instead.
```
cat ~/.ssh/id_rsa.pub | ssh xilinx@pynq "cat >> ~/.ssh/authorized_keys"
```

Use ```df -h``` to check your hard drive space and ```htop``` to check your CPU and RAM usage.
Your board's CPUs are locked at max frequency, so it will heat up regardless of load. I have not found a way to fix this.

# Comfortable wifi setup

Relying on the wifi notebook is not the best, because it does not connect automatically. NetworkManager helps with that.
```
sudo apt install network-manager
```
After a reboot, NetworkManager should start automatically. If not, use ```sudo systemctl start NetworkManager``` and ```sudo systemctl enable NetworkManager```. With the command *nmtui* you get a terminal UI that helps you connect to WIFI networks. It also saves connected networks and auto-connects next time. This can help you avoid needing a USB or Ethernet cable every time.

# Bionic updates repository

Ubuntu usually comes with the updates repository added, but this version does not. This leads to many packages not installing correctly, because of unfulfillable dependencies. Add this to your sources in ```/etc/apt/sources.list.d/multistrap-bionic.list```
```
deb http://ports.ubuntu.com/ubuntu-ports bionic-updates main
```

It should look something like this
```
deb http://ports.ubuntu.com/ubuntu-ports bionic main universe
deb-src http://ports.ubuntu.com/ubuntu-ports bionic main universe
deb http://ports.ubuntu.com/ubuntu-ports bionic-updates main
```

# JupyterLab

To open JupyterLab, go to <http://pynq:9090/lab>.

With this setup, it is possible to manage files, edit code and notebooks in a more IDE-like, intuitive environment, have access to different terminals, and more features similar to VSCode.
Adding a Git client is possible with [jupyterlab-git](https://github.com/jupyterlab/jupyterlab-git)

# VSCode

It is possible to use VSCode with SSH Remote. I had some trouble installing it for the first time, but on a later attempt it worked, this could be because of an update. If there is an issue, delete *.vscode-server/* in your user home folder and try to connect again.

Having VSCode working is very beneficial. Git client, code editors, debugging tools, and file management comes built in, with many extra features available through extensions.

Installing VSCode directly on the board to be used in the GUI environment did not work. There are no error messages but the GUI does not launch.

# ROS

Afterwards, you will want to install ROS. The installation guide tells you to set up your sources with this line.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

The problem with this is that *$(lsb_release -sc)* outputs *glasgow* (the version of the pynq image), while this guide would expect *bionic* for the Ubuntu distribution name, on which the pynq image is based. So replace *$(lsb_release -sc)* with the name of your base ubuntu distro name like so.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
```

After this change you can follow the install [guide](http://wiki.ros.org/melodic/Installation/Ubuntu) normally.

> ROS Melodic still uses Python2, which will have issues using the Python3 pynq libs. Since the latest pynq images are still based on Ubuntu Bionic, ROS Noetic cannot be installed. The only way to use PYNQ with ROS 1 that I found, is to build Melodic from source with Python3 enabled in the build parameters.

[jupyter-ros](https://github.com/RoboStack/jupyter-ros)

## ROS2

Dashing is the latest version that can be installed on the board. Instead of
```
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```
use
```
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list'
```

To set up communication between PX4 and ROS2 follow [this guide](https://dev.px4.io/master/en/middleware/micrortps.html).
This requires you to build *px4_ros_com* from source.
As part of that, you will need to build FastRTPS from source, and you will need to install a JDK and Gradle for that.
The guide suggests using JDK 8, but the older version has compatibility issues with CA certificates.
To get around that issue, it is recommended that you remove all java versions, as well as the java-common package, and reinstall JDK 8.

```
sudo apt purge openjdk*
sudo apt java-common
sudo apt install openjdk-8-jdk
```

After these steps, building with gradle should run without any issues.

Looking at the commands run in the dockerfiles made by the PX4 team may also help. Check out [base](https://hub.docker.com/r/px4io/px4-dev-base-bionic/dockerfile), [simulation](https://hub.docker.com/r/px4io/px4-dev-simulation-bionic/dockerfile), [melodic](https://hub.docker.com/r/px4io/px4-dev-ros-melodic/dockerfile), and [dashing](https://hub.docker.com/r/px4io/px4-dev-ros2-dashing/dockerfile).

[jupyter-ros2](https://github.com/zmk5/jupyter-ros2/tree/ros2)
