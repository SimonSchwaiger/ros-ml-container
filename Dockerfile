# default platform = no gpu acceleration
# possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit)
ARG GRAPHICS_PLATFORM=cpu

# cpu base image does not need to be modified
FROM ros:noetic-robot-focal as build_cpu

# opensource gpu acceleration needs mesa updates
FROM ros:noetic-robot-focal as build_opensource

ONBUILD RUN apt-get update && apt-get -y install libgl1-mesa-glx libgl1-mesa-dri

# for intel, the generic opencl libraries are installed alongside mesa updates
FROM ros:noetic-robot-focal as build_intel

ONBUILD RUN apt-get update && apt-get -y install \
    libgl1-mesa-glx libgl1-mesa-dri \
    ocl-icd-libopencl1 opencl-headers clinfo ocl-icd-opencl-dev

# if env is set to amdpro, copy amdgpu pro driver into container and install it
FROM ros:noetic-robot-focal as build_amdpro

ONBUILD ENV AMDGPUDRIVERFILE="amdgpu-pro-20.45-1188099-ubuntu-20.04.tar.xz"
ONBUILD ENV AMDGPUDIRNAME="amdgpu-pro-20.45-1188099-ubuntu-20.04"

ONBUILD ADD ./$AMDGPUDRIVERFILE .

# install amdgpu pro components for opencl compute acceleration
ONBUILD RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    pciutils \
    wget \
    build-essential \
    cmake \
    libboost-all-dev \
    apt-utils \
    && dpkg --add-architecture i386

ONBUILD RUN apt-get update && apt-get install -y --no-install-recommends \
            libgl1-mesa-glx libgl1-mesa-dri

# manually install amdgpu pro components alongside amdgpu
# (this hybrid approach has been working best for me so far, until amd can get their rocm drivers under control)
ONBUILD RUN cd $AMDGPUDIRNAME \
    && ./amdgpu-install -y --no-dkms \
    && apt-get install -y \
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
    && dpkg -i libdrm2-amdgpu_*_amd64.deb \
    && cd .. \
    && rm -rf $AMDGPUDIRNAME \
    && rm -rf /var/lib/{apt,dpkg,cache,log} \
    && apt-get install -y ocl-icd-opencl-dev

# install opencl prerequisites
ONBUILD RUN apt-get update && apt-get install -y --no-install-recommends \
    clinfo x11vnc xvfb

# set up base image for nvidia container toolkit 
# according to https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html
FROM nvidia/cuda:11.2.1-runtime-ubuntu20.04 as build_nvidia

# recreate ros-noetic-focal-base image
# https://github.com/osrf/docker_images/blob/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/noetic/ubuntu/focal/ros-core/Dockerfile
ONBUILD RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
ONBUILD RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys and sources
ONBUILD RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
ONBUILD RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ONBUILD ENV LANG C.UTF-8
ONBUILD ENV LC_ALL C.UTF-8

# install bootstrap tools
ONBUILD RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
ONBUILD RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
ONBUILD RUN rosdep init && \
  rosdep update --rosdistro noetic

#TODO link tensorflow to cuda by setting necessary environment variables, need an nvidia gpu to test that however


#############################################################
##########          REAL BUILD STARTS HERE         ##########


# default platform is without gpu acceleration (cpu)
# build image corresponding to selected gpu acceleration
FROM build_${GRAPHICS_PLATFORM}

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop \
    ros-noetic-rviz-visual-tools

# install python3, pip and venv
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-tk \
    && python3 -m venv ~/myenv

# upgrade to latest pip
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install --upgrade pip \
    && deactivate"

# install ros python prerequisites
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install launchpadlib \
    wheel \
    && pip3 install -U rosdep \
    rosinstall_generator \
    wstool \
    rosinstall \
    empy \
    catkin_tools \
    defusedxml \
    && pip3 install --upgrade setuptools \
    && deactivate"

# install plaidml, gym and scipy, tensorflow and matplotlib with gui for machine learning
RUN /bin/bash -c "source ~/myenv/bin/activate \ 
    && pip3 install gym \
    && pip3 install scipy \
    && pip3 install plaidml-keras \
    && pip3 install 'h5py<3.0' \
    && pip3 install matplotlib \
    && deactivate"
    #&& pip3 uninstall -y enum34 \
    #&& pip3 install tensorflow \

# copy over ros packages
COPY ./src /catkin_ws/src

# install ros dependencies
RUN rosdep update && rosdep install --from-paths /catkin_ws/src -i -y --rosdistro noetic

# compile workspace
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