#!/bin/bash
# set environment variable for graphics acceleration in the container
# possible values are cpu (no acceleration), opensource (intel and amd open-source), amdpro (amdgpu-pro), nvidia (container-toolkit)
# if GRAPHICS_PLATFORM is null or not set, use cpu
GRAPHICS_PLATFORM="${GRAPHICS_PLATFORM:-cpu}"
# build container
docker build -t mir_search --build-arg GRAPHICS_PLATFORM=$GRAPHICS_PLATFORM .

echo Using graphics platform $GRAPHICS_PLATFORM

if [ "$GRAPHICS_PLATFORM" == "nvidia" ]; then
    # NVIDIA
    # setup xauth
    XAUTH=/tmp/.docker.xauth
    if [ ! -f $XAUTH ]; then
        xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
        if [ ! -z "$xauth_list" ]; then
            echo $xauth_list | xauth -f $XAUTH nmerge -
        else
            touch $XAUTH
        fi
        chmod a+r $XAUTH
    fi
    # run container with necessary args
    #xhost +
    docker run -it \
                --rm \
                --env="DISPLAY=$DISPLAY" \
                --env="QT_X11_NO_MITSHM=1" \
                --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                --env="XAUTHORITY=$XAUTH" \
                --volume="$XAUTH:$XAUTH" \
                --runtime=nvidia \
                mir_search:latest /bin/bash -c "chmod +x /app/app.sh && /app/app.sh"
elif [ "$GRAPHICS_PLATFORM" == "cpu" ]; then
    # CPU
    # run normally, without passing through any devices
    #xhost +
    docker run -it \
                --rm \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                mir_search:latest /bin/bash -c "chmod +x /app/app.sh && /app/app.sh"
else
    # OPENSOURCE and AMDPRO
    # run container in normal mode but pass through dri and kfd devices
    #xhost +
    docker run -it \
                --rm \
                -e DISPLAY=$DISPLAY \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                --device=/dev/dri \
                --device=/dev/kfd \
                mir_search:latest /bin/bash -c "chmod +x /app/app.sh && /app/app.sh"
fi