# Default platform = opensource gpu acceleration
ARG GRAPHICS_PLATFORM=standard
# Default python version is 3.8
ARG PYTHONVER=3.8

# Create base images based on gpu acceleration
## Nvidia-based container
FROM nvidia/cuda:11.2.1-runtime-ubuntu20.04 as build_nvidia
ONBUILD ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ONBUILD ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

## Intel OneAPI Container
FROM intel/oneapi-aikit:devel-ubuntu20.04 as build_intel

## AMD ROCm Container
FROM rocm/dev-ubuntu-20.04:latest as build_amd

## Generic container with MESA
FROM ubuntu:focal as build_standard
FROM ubuntu:focal as build_opensource

## Generic container for WSL
FROM ubuntu:focal as build_wsl
# Set LD library path as in https://github.com/microsoft/wslg/tree/main/samples/container
ONBUILD ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
#TODO: Install directml runtimes in order to be used from python (e.g. tensorflow-directml)
# Without the runtime, only OpenGL is really useful - but it's better than nothing

#############################################################
##########          REAL BUILD STARTS HERE         ##########

## Build container from specified source
FROM build_${GRAPHICS_PLATFORM}

ENV DEBIAN_FRONTEND="noninteractive"

## Recreate ROS noetic base image
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    if ! [ -f /etc/localtime ]; then ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime; fi && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata

# Install prerequisites
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    software-properties-common \
    build-essential

# Setup sources.list and keys for ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install mesa
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    libgl1-mesa-glx libgl1-mesa-dri

# Install ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-rviz ros-noetic-rqt* ros-noetic-rosbridge-server\
    ros-noetic-gmapping ros-noetic-dwa-local-planner ros-noetic-joint-state-publisher-gui

# install python3, pip and venv
# you can change your preferred python version here and it will be installed from the deadsnakes ppa
# some tensorflow implementations (such as gym baselines 2) will require python 3.7
# Forward PYTHONVER argument to the current container
ARG PYTHONVER
ARG GRAPHICS_PLATFORM

RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository -y ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y python$PYTHONVER python$PYTHONVER-dev python$PYTHONVER-tk

RUN apt-get update && apt-get install -y cmake libopenmpi-dev zlib1g-dev imagemagick

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-tk \
    && pip3 install virtualenv

# Create virtualenv using the correct python interpreter
# Intel is an edgecase here, since they offer a custom interpreter in the intel distribution for python
RUN if [ "$GRAPHICS_PLATFORM" = "intel" ]; then \
    virtualenv -p /opt/intel/oneapi/intelpython/latest/bin/python ~/myenv; else \
    virtualenv -p /usr/bin/python$PYTHONVER ~/myenv; fi

# upgrade to latest pip
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install --upgrade pip"

# install ros python prerequisites
# twisted, openssl, autobahn, pymongo and Pillow are there to enable rosbridge server
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install launchpadlib \
    wheel \
    && pip3 install rosdep \
    rosinstall_generator \
    wstool \
    rosinstall \
    empy \
    catkin_tools \
    defusedxml \
    numpy \
    twisted pyOpenSSL autobahn pymongo Pillow service-identity \
    && pip3 install --upgrade setuptools"

# Install required python packages
ADD ./requirements.txt .

RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install -r requirements.txt"

# Copy ROS packages for compilation in container
COPY ./src /catkin_ws/src

# Install ros dependencies
RUN apt-get update && apt-get install -y --no-install-recommends python3-rosdep python3-empy && rosdep init
RUN apt-get update && rosdep update && rosdep install --from-paths /catkin_ws/src -i -y --rosdistro noetic

# Compile workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd catkin_ws \
    && catkin_make -DPYTHON_EXECUTABLE=~/myenv/bin/python"

# Remove src folder used for compilation, since the real src folder will be mounted at runtime
RUN rm -rf /catkin_ws/src

# Cleanup
RUN rm -rf /var/lib/apt/lists/*

# Add ROS sourcing to bashrc for interactive debugging
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source ~/myenv/bin/activate" >> ~/.bashrc

# Set shell env variable for jupyterlab (this fixes autocompletion in web-based shell)
ENV SHELL=/bin/bash

# Add entrypoint
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
