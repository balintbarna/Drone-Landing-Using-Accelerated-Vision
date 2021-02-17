FROM ros:noetic-perception-focal

ARG DEBIAN_FRONTEND=noninteractive
ENV BOARD=Ultra96

RUN apt-get update && apt-get install -y \
        git \
        ros-noetic-mavros-extras \
        python3-pip \
        nano \
        mc \
        usbutils \
        libdrm-dev \
    && rm -rf /var/lib/apt/lists/*

ARG CFLAGS=--sysroot=/
ARG CXXFLAGS=$CFLAGS

WORKDIR /root
RUN mkdir jupyter_notebooks
ENV PYNQ_JUPYTER_NOTEBOOKS=/root/jupyter_notebooks

RUN git clone --recursive https://github.com/Xilinx/PYNQ.git \
    && cd PYNQ/sdbuild/packages/libsds/libcma/ \
    && make install \
    && cd /root \
    && rm -rf PYNQ/

WORKDIR $PYNQ_JUPYTER_NOTEBOOKS

RUN pip3 install \
        jupyterlab \
        pylint \
        cffi \
        pynq \
        pynq-dpu \
    && pynq get-notebooks pynq-dpu -p .

WORKDIR /root

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && bash ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

RUN echo "source /ros_entrypoint.sh" >> /etc/bash.bashrc
RUN echo "set +e" >> /etc/bash.bashrc