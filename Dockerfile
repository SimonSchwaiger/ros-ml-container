# default platform = no gpu acceleration
# possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit)
ARG GRAPHICS_PLATFORM=cpu

# cpu base image does not need to be modified
FROM ros:noetic-robot-focal as build_cpu
ONBUILD ENV DEBIAN_FRONTEND="noninteractive" 

# opensource gpu acceleration needs mesa updates
FROM ros:noetic-robot-focal as build_opensource
ONBUILD ENV DEBIAN_FRONTEND="noninteractive" 

ONBUILD RUN apt-get update && apt-get -y install libgl1-mesa-glx libgl1-mesa-dri

# for intel, the generic opencl libraries are installed alongside mesa updates
FROM ros:noetic-robot-focal as build_intel
ONBUILD ENV DEBIAN_FRONTEND="noninteractive" 

ONBUILD RUN apt-get update && apt-get -y install \
    libgl1-mesa-glx libgl1-mesa-dri \
    ocl-icd-libopencl1 opencl-headers clinfo ocl-icd-opencl-dev intel-opencl-icd

# if env is set to amdpro, copy amdgpu pro driver into container and install it
FROM ros:noetic-robot-focal as build_amdpro
ONBUILD ENV DEBIAN_FRONTEND="noninteractive" 

ONBUILD ENV AMDGPUDRIVERFILE="amdgpu-pro-21.20-1271047-ubuntu-20.04.tar.xz"
ONBUILD ENV AMDGPUDIRNAME="amdgpu-pro-21.20-1271047-ubuntu-20.04"

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
ONBUILD ENV DEBIAN_FRONTEND="noninteractive" 

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
    python3-vcstools

# install ros packages
ONBUILD RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* 

# bootstrap rosdep
ONBUILD RUN rosdep init && \
  rosdep update --rosdistro noetic

#TODO link tensorflow to cuda by setting necessary environment variables, need an nvidia gpu to test that however
# thanks ritschie
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# wsl2 gpu acceleration needs mesa updates
FROM ros:noetic-robot-focal as build_wsl2
ONBUILD ENV DEBIAN_FRONTEND="noninteractive" 

ONBUILD RUN apt-get update && apt-get -y install libgl1-mesa-glx libgl1-mesa-dri
#TODO: Install directml runtimes in order to be used from python (e.g. tensorflow-directml)
# Without the runtime, only OpenGL is really useful - but it's better than nothing

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
    && pip3 install --upgrade pip"

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
    && pip3 install --upgrade setuptools"

# install machine learning and other desired python3 modules using the requirements.txt file
# if nvidia is used, i recommend tensorflow + keras; for everyone else, plaidml + keras
# plaidml and tensorflow cannot co-exist
ADD ./requirements.txt .

RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install -r requirements.txt"

# copy over ros packages from ros workspace
COPY ./src /catkin_ws/src

# install ros dependencies
RUN rosdep update && rosdep install --from-paths /catkin_ws/src -i -y --rosdistro noetic

# compile workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd catkin_ws \
    && catkin_make "

# cleanup
RUN rm -rf /var/lib/apt/lists/*

