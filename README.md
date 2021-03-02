# ROS ML Container
ROS Noetic Container for Machine Learning with switchable graphics acceleration methods.

# Install docker
https://docs.docker.com/engine/install/ubuntu/

# Set up docker to run without sudo
sudo groupadd docker && sudo usermod -aG docker $USER

Paste these commands and log out and in.

# Run
The software is run by executing the builandrun.sh script. This script will build a docker container with all the necessary files and start 'app/app.sh'.

The first build will take quite a while, but consecutive builds will be faster, since docker caches each stage of the build.

By default, no GPU acceleration is applied. However if desired, the type of GPU acceleration can be specified with the GRAPHICS_PLATFORM environment variable before starting buildandrun.sh.

Please note that changing the used graphics platform will cause a complete image rebuild. Therefore, the first build will take a while.

Example usage:

'GRAPHICS_PLATFORM=amdpro ./buildandrun.sh'

Possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit).
For use of the amdgpu-pro driver, amd opencl must run on the host and the amdgpu-pro-20.45* tar file must be downloaded manually and put into the same directory as the dockerfile. For use of cuda, the nvidia proprietary driver must be installed on the host, along with nvidia-docker2.

# Usage

ROS packages are intended to be put into 'src', since this folder is copied into the container and compiled, while 'app' is intended for scripts.

# Cleanup
Remove all images and shutdown containers with these commands:

'docker rm $(docker ps -a -f status=exited -q) && docker rmi -f $(docker images --quiet)'