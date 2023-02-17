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
    argcomplete
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

# Fully install ros2 instead of bootstrapping it
RUN apt-get update && apt-get install -y ros-humble-desktop

# Also install ignitio-edifice for gazebo simulation
#TODO: Change to Webots? https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots.html
#RUN apt-get update && apt-get install -y wget
#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
#RUN http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
#RUN apt-get update && apt-get install -y ignition-edifice

# Install Turtlebot 4 Packages for testing
#RUN apt-get update && apt-get install -y ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes

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
#RUN /bin/bash -c "source ~/myenv/bin/activate \
#    && pip3 install launchpadlib \
#    wheel \
#    && pip3 install rosdep \
#    rosinstall_generator \
#    wstool \
#    rosinstall \
#    empy \
#    catkin_tools \
#    defusedxml \
#    && pip3 install --upgrade setuptools"

# Install required python packages
ADD ./requirements.txt .

RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install -r requirements.txt"

# Copy ROS packages for compilation in container
COPY ./src /opt/$ROS2_WS/src

# Install ros dependencies
RUN apt-get update && rosdep update && rosdep install --from-paths /opt/$ROS2_WS/src -i -y --rosdistro humble

# Compile workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
    && cd /opt/$ROS2_WS \
    && colcon build --symlink-install"

# Remove src folder used for compilation, since the real src folder will be mounted at runtime
RUN rm -rf /opt/$ROS2_WS/src

# Cleanup
RUN rm -rf /var/lib/apt/lists/*

# Add ROS and venv sourcing to bashrc for interactive debugging
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ~/myenv/bin/activate" >> ~/.bashrc

# Add entrypoint
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
