FROM ros:noetic-perception-focal

RUN apt-get update && apt-get install -y \
        git \
        ros-noetic-mavros \
        ros-noetic-mavros-msgs \
        ros-noetic-mavros-extras \
        python3-pip \
        mc \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
        jupyterlab \
        pylint

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

RUN echo "source /ros_entrypoint.sh" >> /etc/bash.bashrc
RUN echo "set +e" >> /etc/bash.bashrc