FROM ros:noetic-perception-focal

RUN apt-get update && apt-get install -y \
        git \
        ros-noetic-mavros \
        ros-noetic-mavros-msgs \
        ros-noetic-mavros-extras \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
        jupyterlab
