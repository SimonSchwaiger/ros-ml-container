FROM ros:noetic-robot-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1* \
    ros-noetic-rviz-visual-tools
    #&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    && python3 -m venv ~/myenv

# install python prerequisites
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install launchpadlib \
    wheel \
    && pip3 install -U rosdep \
    rosinstall_generator \
    wstool \
    rosinstall \
    empy \
    catkin_tools \
    && pip3 install --upgrade setuptools \
    && pip3 install gym scipy sympy pyserial \
    && deactivate"

# install amdgpu pro components for opencl compute acceleration
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    pciutils \
    wget \
    build-essential \
    cmake \
    libboost-all-dev \
    apt-utils \
    && dpkg --add-architecture i386

ENV AMDGPUDRIVERFILE="amdgpu-pro-20.45-1188099-ubuntu-20.04.tar.xz"
ENV AMDGPUDIRNAME="amdgpu-pro-20.45-1188099-ubuntu-20.04"

ADD ./$AMDGPUDRIVERFILE .

RUN cd $AMDGPUDIRNAME \
    && ./amdgpu-install -y --no-dkms \
    && apt install -y \
    amdgpu-core \
    comgr-amdgpu-pro \
    hip-rocr-amdgpu-pro \
    hsa-runtime-rocr-amdgpu \
    hsakmt-roct-amdgpu \
    libdrm-amdgpu-amdgpu1 \
    libdrm-amdgpu-common \
    libdrm2-amdgpu \
    ocl-icd-libopencl1-amdgpu-pro \
    opencl-rocr-amdgpu-pro \
    && dpkg -i opencl-rocr-amdgpu-pro_*_amd64.deb \
    && dpkg -i opencl-rocr-amdgpu-pro-dev_*_amd64.deb \
    && dpkg -i rocm-device-libs-amdgpu-pro_*_amd64.deb \
    && dpkg -i hsa-runtime-rocr-amdgpu_*_amd64.deb \
    && dpkg -i hsa-runtime-rocr-amdgpu-dev_*_amd64.deb \
    && dpkg -i hsakmt-roct-amdgpu_*_amd64.deb \
    && dpkg -i hsakmt-roct-amdgpu-dev_*_amd64.deb \
    && dpkg -i hip-rocr-amdgpu-pro_*_amd64.deb \
    && dpkg -i comgr-amdgpu-pro_*_amd64.deb \
    && dpkg -i comgr-amdgpu-pro-dev_*_amd64.deb \
    && dpkg -i opencl-orca-amdgpu-pro-icd_*_amd64.deb \
    && dpkg -i libdrm-amdgpu-amdgpu1_*_amd64.deb \
    && cd .. \
    && rm -rf $AMDGPUDIRNAME \
    && rm -rf /var/lib/{apt,dpkg,cache,log} \
    && apt install -y ocl-icd-opencl-dev

# install plaidml
RUN apt-get update && apt-get install -y --no-install-recommends \
    clinfo x11vnc xvfb

RUN /bin/bash -c "source ~/myenv/bin/activate \ 
    && pip3 install -U plaidml-keras \
    && pip3 uninstall -y enum34 \
    && deactivate"

COPY .plaidml ~/

# install matplotlib with gui
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && apt -y install python3-tk \
    && pip3 install matplotlib \
    && deactivate"

# install ros packages
COPY ./src /catkin_ws/src

RUN /bin/bash -c "source ~/myenv/bin/activate \ 
    && pip3 install -e /catkin_ws/src/fhtw3dof/gym-fhtw3dof \
    && deactivate"

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd catkin_ws \
    && catkin_make "

# get application
COPY ./app /app

# xvfb-run roslaunch mairobot MairobotClickAndGo.launch

# https://stackoverflow.com/questions/48561981/activate-python-virtualenv-in-dockerfile#:~:text=virtualenv%20is%20used%20for%20dependency,between%20containers%20and%20between%20applications.

# http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/

#apt install -y amdgpu-core comgr-amdgpu-pro hip-rocr-amdgpu-pro hsa-runtime-rocr-amdgpu hsakmt-roct-amdgpu libdrm-amdgpu-amdgpu1 libdrm-amdgpu-common libdrm2-amdgpu ocl-icd-libopencl1-amdgpu-pro opencl-rocr-amdgpu-pro





#   && apt-get -y remove ca-certificates curl xz-utils \
#   && apt-get -y autoremove && apt-get clean autoclean \

# https://github.com/BaileySN/Docker_AMDGPU_Base_Image/blob/master/Dockerfile

# https://askubuntu.com/questions/809081/how-to-run-opencl-program-in-docker-container-with-amdgpu-pro

#RUN ./$AMDGPUDIRNAME/amdgpu-pro-install -y --opencl=pal && \
#    rm -rf $AMDGPUDIRNAME && \
#    apt-get -y remove ca-certificates curl xz-utils && \
#    apt-get -y autoremove && apt-get clean autoclean && \
#    rm -rf /var/lib/{apt,dpkg,cache,log}

#RUN apt-get install opencl-amdgpu-pro -y

#RUN wget -qO - http://repo.radeon.com/rocm/apt/debian/rocm.gpg.key | apt-key add - && \
#	sh -c 'echo deb [arch=amd64] http://repo.radeon.com/rocm/apt/debian/ focal main > /etc/apt/sources.list.d/rocm.list'