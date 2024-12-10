#!/bin/bash

# Default configuration. Can be overwritten with env variables
GRAPHICS_PLATFORM="${GRAPHICS_PLATFORM:-opensource}"
DOCKER_RUN_ARGS="${DOCKER_RUN_ARGS:-"-p 8888:8888"}"
BUILD_LOCAL="${BUILD_LOCAL:-false}"
SKIP_COMPILE="${SKIP_COMPILE:-false}" # 
ROS_DISTRO="${ROS_DISTRO:-"humble"}"
CONTAINER_NAME="${CONTAINER_NAME:-ros_ml_container}"

# Check if container is already running and attach if it is
if [ "$(docker ps -aq --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    echo "Detected running container instance. Attaching to the running container"
    docker exec -it ros_ml_container bash $@
    exit 0
fi

# Function to print to stderr: https://stackoverflow.com/questions/2990414/echo-that-outputs-to-stderr
# Colouring: https://superuser.com/questions/542074/how-can-i-customize-the-color-of-error-messages-in-bash
echoerr_util() { echo "$@" 1>&2; }
echoerr() { echoerr_util "$@" 2> >(while read line; do echo -e "\e[01;31m$line\e[0m" >&2; done); }

# Prevent running as root and check if docker can be run without root privileges
# Based on: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/scripts/run_dev.sh
if [[ $(id -u) -eq 0 ]]; then
    echoerr "The buildandrun script should not be executed as root"
    exit 1
fi

RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    echoerr "Please add your user to the docker group to run docker without sudo"
    exit 1
fi

# Check if src and lab folders exist, if not they will be created
# The lab folder allows the jupyterlab config to persist between sessions
mkdir -p src
mkdir -p app
mkdir -p lab/workspaces

# Check if requirements file exists and create it with example packages if it isn't
if [ ! -f  "requirements.txt" ]; then
    echo "jupyterlab # put your required Python3 packages here. They will be installed using Pip!" > requirements.txt
fi

# Lookup correct configuration
IMAGE_CONFIG=$(python3 baseimages/get_base_dockerfile.py baseimages/images.json $ROS_DISTRO $GRAPHICS_PLATFORM)

# Aborts script if build is not successful
check_docker_build_success() {
    if [ $? -ne 0 ]; then
        echoerr "Failed to build image. Please check docker build output"
        exit 1
    fi
};

# Check ci images and determine, whether or not the specified config exists in the cloud
REGISTRY_AVAILABLE=$(python3 baseimages/check_tag_existance.py .github/workflows/ci_images.json $ROS_DISTRO $GRAPHICS_PLATFORM)

# Check if container should be built locally (if GRAPHICS_PLATFORM has been changed or local build explicitly requested)
if [ "$BUILD_LOCAL" == "false" ] && [ "$REGISTRY_AVAILABLE" == "true" ]; then
    ## Pull remote container
    docker pull ghcr.io/simonschwaiger/ros-ml-container:$(echo $IMAGE_CONFIG| jq -r '.TAG')
    docker tag ghcr.io/simonschwaiger/ros-ml-container:$(echo $IMAGE_CONFIG| jq -r '.TAG') ros_ml_container:$(echo $IMAGE_CONFIG| jq -r '.TAG')
else
    ## Prepare baseimage (to prevent redundant downloads)
    # Pull correct image based on base image and corresponding dockerfile
    docker build -t ros_ml_container:baseimage \
    --build-arg BASEIMAGE=$(echo $IMAGE_CONFIG| jq -r '.BASEIMAGE') \
    -f baseimages/$(echo $IMAGE_CONFIG| jq -r '.BASE_DOCKERFILE') \
    .
    check_docker_build_success
    ## Local container build -> Tag as specified in json
    docker build -t ros_ml_container:$(echo $IMAGE_CONFIG| jq -r '.TAG') \
    --build-arg GRAPHICS_PLATFORM=$GRAPHICS_PLATFORM \
    --build-arg PYTHONVER=$(echo $IMAGE_CONFIG| jq -r '.PYTHONVER') \
    --build-arg ROS_DISTRO=$(echo $IMAGE_CONFIG| jq -r '.ROS_DISTRO') \
    -f distroimages/$(echo $IMAGE_CONFIG| jq -r '.DOCKERFILE') \
    .
    check_docker_build_success
fi

if [ "$SKIP_COMPILE" == "false" ]; then
    # Build catkin packages on top of remote or local container
    docker build -t ros_ml_container \
    --build-arg TAG=$(echo $IMAGE_CONFIG| jq -r '.TAG') \
    --build-arg ROS_DISTRO=$(echo $IMAGE_CONFIG| jq -r '.ROS_DISTRO') \
    -f Dockerfile.remote \
    .
    check_docker_build_success
else
    docker tag ros_ml_container:$(echo $IMAGE_CONFIG| jq -r '.TAG') ros_ml_container:latest
fi

# Set xhost permissions for docker
# TODO better solution
# https://unix.stackexchange.com/questions/330366/how-can-i-run-a-graphical-application-in-a-container-under-wayland
xhost +local:docker

ROS2_WS=$(echo $IMAGE_CONFIG| jq -r '.ROS2_WS')

# Set up docker run args for gpu forwarding
DOCKER_ARGS+=("-e DISPLAY=$DISPLAY")
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $PWD/src:$ROS2_WS/src")
DOCKER_ARGS+=("-v $PWD/app:/app")
DOCKER_ARGS+=("-v $PWD/lab:/root/.jupyter/lab")

if [ "$GRAPHICS_PLATFORM" == "nvidia" ]; then
    # NVIDIA
    # Enable all gpus and run privileged for container toolkit
    DOCKER_ARGS+=("--gpus all")
    DOCKER_ARGS+=("--privileged")
    DOCKER_ARGS+=("--runtime nvidia")

elif [ "$GRAPHICS_PLATFORM" == "cpu" ]; then
    # CPU
    # No extra args currently required
    DOCKER_ARGS+=("")

elif [ "$GRAPHICS_PLATFORM" == "amd" ]; then
    # ROCm
    # Add user to video group and pass through dri and kfd devices
    DOCKER_ARGS+=("--device=/dev/dri")
    DOCKER_ARGS+=("--device=/dev/kfd")
    DOCKER_ARGS+=("--security-opt seccomp=unconfined --group-add video")
    DOCKER_ARGS+=("--cap-add=SYS_PTRACE --ipc=host --shm-size 8G")

elif [ "$GRAPHICS_PLATFORM" == "wsl2" ]; then
    # WSL2
    # Run container in similarly to opensource, pass through dri device for OpenGL and dxg device for DirectX 12
    # https://fossbytes.com/directx-on-linux-wsl2-support-windows-10/
    DOCKER_ARGS+=("--privileged")
    DOCKER_ARGS+=("--device=/dev/dri:/dev/dri")
    DOCKER_ARGS+=("--device=/dev/dxg:/dev/dxg")

elif [ "$GRAPHICS_PLATFORM" == "opensource" ]; then
    # OPENSOURCE
    # Pass through dri device for OpenGL
    DOCKER_ARGS+=("--device=/dev/dri:/dev/dri")

elif [ "$GRAPHICS_PLATFORM" == "intel" ]; then
    # INTEL
    # Pass through dri device for OpenGL
    DOCKER_ARGS+=("--device=/dev/dri:/dev/dri")
fi

# Remove existing container instances to prevent conflicts when starting
if [ "$(docker ps -a --quiet --filter status=exited --filter name=ros_ml_container)" ]; then
    docker rm ros_ml_container > /dev/null
fi

# Start container
echo Using graphics platform $GRAPHICS_PLATFORM
echo Using python version $(echo $IMAGE_CONFIG| jq -r '.PYTHONVER')

docker run -it --rm --name $CONTAINER_NAME \
            $DOCKER_RUN_ARGS ${DOCKER_ARGS[@]} \
            ros_ml_container:latest /bin/bash -c "chmod +x /app/app.sh && (cd app ; ./app.sh)"
