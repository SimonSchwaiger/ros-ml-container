#!/bin/bash
#docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device /dev/dri:/dev/dri fhtw3dof:latest /bin/bash -c "chmod +x /app/app.sh && ./app/app.sh"

docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri --device=/dev/kfd fhtw3dof:latest /bin/bash -c "chmod +x /app/app.sh && ./app/app.sh"
