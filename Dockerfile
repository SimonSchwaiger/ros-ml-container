# Default platform = opensource gpu acceleration
ARG GRAPHICS_PLATFORM=standard
# Default python version is 3.8
ARG PYTHONVER=3.10

# Create base images based on gpu acceleration
## Nvidia-based container
FROM nvidia/cuda:11.8.0-runtime-ubuntu22.04 as build_nvidia
ONBUILD ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ONBUILD ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

## Intel OneAPI Container
FROM intel/oneapi-aikit:2023.1.0-devel-ubuntu22.04 as build_intel

## AMD ROCm Container
FROM rocm/dev-ubuntu-22.04:latest as build_amd

## Generic container with MESA
FROM ubuntu:jammy as build_standard
FROM ubuntu:jammy as build_opensource

## Generic container for WSL
FROM ubuntu:jammy as build_wsl
# Set LD library path as in https://github.com/microsoft/wslg/tree/main/samples/container
ONBUILD ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
#TODO: Install directml runtimes in order to be used from python (e.g. tensorflow-directml)
# Without the runtime, only OpenGL is really useful - but it's better than nothing

#############################################################
##########          REAL BUILD STARTS HERE         ##########

## Build container from specified source
FROM build_${GRAPHICS_PLATFORM}
LABEL org.opencontainers.image.source="https://github.com/SimonSchwaiger/ros-ml-container"

ENV DEBIAN_FRONTEND="noninteractive"

# Install mesa for GUI
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    libgl1-mesa-glx libgl1-mesa-dri

## Recreate ROS humble devel image
# ------------------------
# https://github.com/osrf/docker_images/blob/master/ros2/source/devel/Dockerfile
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    bash-completion \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-flake8 \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-docstrings \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pip \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
    argcomplete pytest
# add pytest here manually as a workaround for intel-based builds

# This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# clone source
ENV ROS2_WS /opt/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

# build source
RUN colcon \
    build \
    --cmake-args \
      -DSECURITY=ON --no-warn-unused-cli \
    --symlink-install

# setup bashrc
RUN cp /etc/skel/.bashrc ~/

WORKDIR /
## Official ROS image recreated
# ------------------------

# Set ROS and ignition versions and install them
ENV ROS_DISTRO humble
ENV IGNITION_VERSION fortress

# Fully install ros2 instead of bootstrapping it and install rqt for debugging and rosbridge for web-based visualisation
RUN apt-get update && apt-get install -y ros-$ROS_DISTRO-ros-base ros-dev-tools ros-$ROS_DISTRO-rqt* ros-$ROS_DISTRO-rosbridge-server

# Install ignition gazebo
#RUN apt-get install -y ros-$ROS_DISTRO-ros-ign ignition-$IGNITION_VERSION

# Update os and ros packages due to fix buggy opengl
RUN apt-get update && apt-get upgrade -y

# install python3, pip and venv
# you can change your preferred python version here and it will be installed from the deadsnakes ppa
# some tensorflow implementations (such as gym baselines 2) will require python 3.7
# Forward PYTHONVER argument to the current container
ARG PYTHONVER
ARG GRAPHICS_PLATFORM

RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository -y ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y python$PYTHONVER python$PYTHONVER-dev python$PYTHONVER-tk

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    cmake \
    libopenmpi-dev \
    zlib1g-dev \
    imagemagick

RUN pip3 install virtualenv

# Create virtualenv using the correct python interpreter
# Intel is an edgecase here, since they offer a custom interpreter in the intel distribution for python
RUN if [ "$GRAPHICS_PLATFORM" = "intel" ]; then \
    virtualenv -p /opt/intel/oneapi/intelpython/latest/bin/python ~/myenv; else \
    virtualenv -p /usr/bin/python$PYTHONVER ~/myenv; fi

# Upgrade to latest pip
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install --upgrade pip"

# Install ros python prerequisites
# Pytest is explicitly installed to handle the intel edgecase
# Netifaces, pymongo and Pillow are installed for rosbridge
RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install launchpadlib \
    rosinstall_generator \
    rosinstall \
    empy \
    catkin_tools \
    lark \
    lxml \
    pytest \
    numpy \
    netifaces pymongo Pillow \
    && pip3 install --upgrade setuptools"

# Install required python packages
ADD ./requirements.txt .

RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install -r requirements.txt"

# Copy ROS packages for compilation in container
COPY ./src /opt/$ROS2_WS/src

# Install ros dependencies
RUN apt-get update && rosdep update && rosdep install --from-paths /opt/$ROS2_WS/src -i -y --rosdistro $ROS_DISTRO

# Compile workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd /opt/$ROS2_WS \
    && colcon build --symlink-install"

# Remove src folder used for compilation, since the real src folder will be mounted at runtime
RUN rm -rf /opt/$ROS2_WS/src

# Cleanup
RUN rm -rf /var/lib/apt/lists/*

# Add ROS and venv sourcing to bashrc for interactive debugging
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo ". /opt/ros2_ws/install/local_setup.bash" >> ~/.bashrc
RUN echo "source ~/myenv/bin/activate" >> ~/.bashrc

# Set shell env variable for jupyterlab (this fixes autocompletion in web-based shell)
ENV SHELL=/bin/bash

# Add entrypoint
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
