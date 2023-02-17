#!/bin/bash
# set environment variable for graphics acceleration in the container
# possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit)
# if GRAPHICS_PLATFORM is null or not set, use cpu
GRAPHICS_PLATFORM="${GRAPHICS_PLATFORM:-opensource}"
PYTHONVER="${PYTHONVER:-3.10}"
DOCKER_RUN_ARGS="${DOCKER_RUN_ARGS:-"-p 8888:8888"}"

# check if src folder exists, if not it will be created
mkdir -p src

# check if requirements file exists and create it with example packages if it isn't
if [ ! -f  "requirements.txt" ]; then
    echo "jupyterlab # put your required Python3 packages here. They will be installed using Pip!" > requirements.txt
fi

# build container
docker build -t ros_ml_container \
--build-arg GRAPHICS_PLATFORM=$GRAPHICS_PLATFORM \
--build-arg PYTHONVER=$PYTHONVER \
.

echo Using graphics platform $GRAPHICS_PLATFORM
echo Using python version $PYTHONVER

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
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/src":/opt/ros2_ws/src \
                -v "$PWD/app":/app \
                $DOCKER_RUN_ARGS \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
elif [ "$GRAPHICS_PLATFORM" == "cpu" ]; then
    # CPU
    # run normally, without passing through any devices
    docker run -it \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/src":/opt/ros2_ws/src \
                -v "$PWD/app":/app \
                $DOCKER_RUN_ARGS \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
elif [ "$GRAPHICS_PLATFORM" == "amdpro" ]; then
    # AMDPRO
    # run container in normal mode but pass through dri and kfd devices
    docker run -it \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/src":/opt/ros2_ws/src \
                -v "$PWD/app":/app \
                $DOCKER_RUN_ARGS \
                --device=/dev/dri \
                --device=/dev/kfd \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
elif [ "$GRAPHICS_PLATFORM" == "wsl2" ]; then
    # WSL2
    # run container in normal mode and pass through dri device for OpenGL and dxg Device for DirectX 12
    # https://fossbytes.com/directx-on-linux-wsl2-support-windows-10/
        docker run -it \
                --rm \
                --name ros_ml_container \
                --privileged \
                -e DISPLAY=$DISPLAY \
                --device=/dev/dri:/dev/dri \
                --device=/dev/dxg:/dev/dxg \
                -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -v "$PWD/src":/opt/ros2_ws/src \
                -v "$PWD/app":/app \
                $DOCKER_RUN_ARGS \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
else
    # OPENSOURCE and INTEL
    # run container in normal mode but pass through dri device
    docker run -it \
                --rm \
                --name ros_ml_container \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$PWD/src":/opt/ros2_ws/src \
                -v "$PWD/app":/app \
                $DOCKER_RUN_ARGS \
                --device=/dev/dri \
                ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
fi
