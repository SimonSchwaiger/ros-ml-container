#!/bin/bash
# set environment variable for graphics acceleration in the container
# possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit)
# if GRAPHICS_PLATFORM is null or not set, use cpu
GRAPHICS_PLATFORM="${GRAPHICS_PLATFORM:-cpu}"

# check if src folder exists, if not it will be created
mkdir -p src

# check if requirements file exists and create it with example packages if it isn't
if ! -f  "requirements.txt"; then
    echo "numpy # put your required Python3 packages here. They will be installed using Pip!" > requirements.txt
fi

# if amdpro driver is used, download it if it is not already present
AMDPROFILE="amdgpu-pro-21.20-1271047-ubuntu-20.04.tar.xz"
if [ "$GRAPHICS_PLATFORM" == "amdpro" ] && [ ! -f "$AMDPROFILE" ]; then
    wget --referer=http://support.amd.com  https://drivers.amd.com/drivers/linux/amdgpu-pro-21.20-1271047-ubuntu-20.04.tar.xz
fi

# build container
docker build -t ros_ml_container --build-arg GRAPHICS_PLATFORM=$GRAPHICS_PLATFORM .

echo Using graphics platform $GRAPHICS_PLATFORM

# Set xhost permissions for docker
# TODO better solution
# https://unix.stackexchange.com/questions/330366/how-can-i-run-a-graphical-application-in-a-container-under-wayland
xhost +local:docker

if [ "$GRAPHICS_PLATFORM" == "nvidia" ]; then
    # NVIDIA
    # run container with necessary args
    docker run -it \
                --gpus all \
                --privileged \
                --network host \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
elif [ "$GRAPHICS_PLATFORM" == "cpu" ]; then
    # CPU
    # run normally, without passing through any devices
    docker run -it \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
elif [ "$GRAPHICS_PLATFORM" == "amdpro" ]; then
    # run container in normal mode but pass through dri and kfd devices
    docker run -it \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                --device=/dev/dri \
                --device=/dev/kfd \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
else
    # OPENSOURCE and INTEL
    # run container in normal mode but pass through dri device
    docker run -it \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                --device=/dev/dri \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
fi
