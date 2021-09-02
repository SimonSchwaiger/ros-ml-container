# ROS ML Container
ROS Noetic container for machine learning with switchable graphics acceleration methods. 

This repository provides a full [Robot Operating System (ROS)](https://www.ros.org/) workspace without actually installing ROS on your machine. The goal of this workspace is to allow for convenient development of somewhat portable ROS applications that run within a container. In order to speed up robot simulation as well as machine learning applications within the container, the graphics card (GPU) of your PC can be passed through to the container. The type of GPU acceleration depends on your graphics card and operating system.

These GPU acceleration methods are supported:

- __cpu__: No acceleration is applied. This method is good for unsupported OS/GPU combinations or for debugging if GPU acceleration is not working properly.

- __opensource__: Passes through open source drivers of your video card on Linux. This works for Nvidia, AMD and Intel GPUs and passes through OpenGL capabilities. Open source drivers currently do not allow acceleration of machine learning tasks, since they either require __CUDA__ on Nvidia or __OpenCL__ on AMD and Intel. However, using this method will accelerate ROS simulations and visualisations. __If you are on Linux and not sure what GPU acceleration your system supports, it is probably this one.__

- __intel__: Same as opensource, but it will also install and pass through the Intel OpenCL device. TODO Installation on host

- __amdpro__: Installs the [amdgpu-pro]() driver within the container in order to register your AMD GPU as an OpenCL device. In addition to OpenGL, this method also passes through OpenCL in order to enable GPU accelerated machine learning.

- __nvidia__: Passes through OpenGL and [CUDA](https://developer.nvidia.com/cuda-downloads) capabilities using the [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

- __wsl2__: Passes through the [Windows Subsystem for Linux (WSL2)](https://docs.microsoft.com/en-us/windows/wsl/about) virtual GPU to the container in order to allow for OpenGL, CUDA and DirectML acceleration. __This method is required in order to get GPU acceleration working when running Docker from Windows using the WSL2 backend. TODO - This does not work yet.__


## Prerequisites

You will need to have [Docker](https://www.docker.com/) installed on your system. On Linux, docker can be [installed natively](https://docs.docker.com/engine/install/ubuntu/), while on Windows, it is required to use the [WSL2 backend](https://docs.docker.com/desktop/windows/install/). 

- __Linux__:
  * [Install Docker natively.](https://docs.docker.com/engine/install/ubuntu/)
  * Allow Docker to run without _sudo_ using the _sudo groupadd docker && sudo usermod -aG docker $USER_ command. Afterwards, log out and in again to apply changes.

- __Windows__:
  * Windows 10 Update 21h1 or newer is required for the Linux GUI to be forwarded.
  * [Install Docker using the WSL2 backend.](https://docs.docker.com/desktop/windows/install/)
  * [Install WSL GPU drivers for Windows](https://docs.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)

Some means of acceleration require more packages to be installed on your host system:

- __opensource__: Open source drivers for your video card (On most distributions, open source drivers are installed for AMD and Intel GPUs by default).

- __intel__: Open source intel drivers with either the unofficial OpenCL loader from your Linux distribution (for example, for ubuntu the unofficial loader can be installed using _sudo apt install ocl-icd-opencl-dev_) or the official [OpenCL runtime for Intel processors](https://software.intel.com/content/www/us/en/develop/articles/opencl-drivers.html) installed.

- __amdpro__: Either the [amdgpu-pro](https://www.amd.com/en/support/kb/release-notes/rn-amdgpu-unified-linux-21-10) driver, the unofficial [opencl-amd aur package](https://aur.archlinux.org/packages/opencl-amd/) on Arch-based Linux distributions or the [ROCm-OpenCL-Runtime](https://github.com/RadeonOpenCompute/ROCm-OpenCL-Runtime) must be installed in order to access your AMD GPU's OpenCL capabilities on the Linux host. __When using this type of acceleration the first time, the amdgpu-pro driver will automatically be downloaded and placed in the same directory as the build script, in order for the driver to be copied into and installed in the docker image.__

- __nvidia__: [Proprietary Nvidia GPU driver](https://www.nvidia.com/de-de/drivers/unix/) and [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed on your system. Pudget Systems has a good [guide](https://www.pugetsystems.com/labs/hpc/Workstation-Setup-for-Docker-with-the-New-NVIDIA-Container-Toolkit-nvidia-docker2-is-deprecated-1568/) for the installation process. 


## Usage

A [Skript](./buildandrun.sh) is provided to automatically build and run the container. The means of GPU acceleration can be passed through using the *GRAPHICS_PLATFORM* environment variable, the default is __cpu__. The first build will take quite a while, but consecutive builds will be faster, since docker caches each stage of the build. However, changing the *GRAPHICS_PLATFORM* will cause a full rebuild of the container.

Example for running the container with acceleration set to __opensource__:
*GRAPHICS_PLATFORM=opensource ./buildandrun.sh*

Upon the first build, an *src* folder and *requirements.txt* are created in the scirpts directory, if they do not already exist. *src* is intended for ROS packages to be placed into. During the build, this folder is copied into the image, dependencies of all packages are installed and the workspace is compiled. The Python3 packages defined in *requirements.txt* are installed using Pip in a virtual environment located at */myenv* within the image. Since the ROS and Python packages are part of the image, changes in *src* or *requirements.txt* cause a partial rebuild and changes made in the container do not carry over from container to host.

The [app](./app) folder is intended to contain configuration files (for example *.rviz*) and scripts. This folder is mounted to the container at */app* and is automatically changed into upon container startup. This means, that scripts can be edited on the host and tested inside the container, without requiring a rebuild. Additionally, saved data and generated plots can be saved in this directory in order to be shared between host and different container sessions.

Containers are automatically deleted after shutdown. The generated images must be [removed manually](https://docs.docker.com/engine/reference/commandline/rmi/).
