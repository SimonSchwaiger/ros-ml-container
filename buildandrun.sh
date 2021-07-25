#!/bin/bash
# set environment variable for graphics acceleration in the container
# possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit)
# if GRAPHICS_PLATFORM is null or not set, use cpu
GRAPHICS_PLATFORM="${GRAPHICS_PLATFORM:-cpu}"

# check if src folder exists, if not it will be created
mkdir -p src

# build container
docker build -t ros_ml_container --build-arg GRAPHICS_PLATFORM=$GRAPHICS_PLATFORM .

echo Using graphics platform $GRAPHICS_PLATFORM

if [ "$GRAPHICS_PLATFORM" == "nvidia" ]; then
    # NVIDIA
    # run container with necessary args
    #xhost + #TODO: check if xhost + is necessary on some os'
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
    #xhost +
    docker run -it \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
elif [ "$GRAPHICS_PLATFORM" == "amdpro" ]; then
    # run container in normal mode but pass through dri and kfd devices
    #xhost +
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
    #xhost +
    docker run -it \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/app":/app \
                --device=/dev/dri \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
fi
