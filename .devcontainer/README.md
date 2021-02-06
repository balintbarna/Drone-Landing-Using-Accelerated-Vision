This docker context is used to build the image based on ROS.

Usage:
- clone repository locally
- clone repository on the board into `~/src`
- build docker image on the board with `docker build -t drone-project:latest .` from `.devcontainer`
- set up SSH key for passwordless connection to the board
- add user to docker group with `sudo adduser xilinx docker` and reboot board
- add `"docker.host": "ssh://xilinx@pynq"` to VSCode `settings.json`
- select `Remote-Containers: Open Folder in Container` in VSCode and choose the local clone of the repository to use its `devcontainer.json` configuration
