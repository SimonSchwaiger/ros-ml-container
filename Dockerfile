FROM ros:noetic-robot-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1* \
    ros-noetic-rviz-visual-tools
    #&& rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    && python3 -m venv ~/myenv

RUN /bin/bash -c "source ~/myenv/bin/activate \
    && pip3 install launchpadlib \
    wheel \
    && pip3 install -U rosdep \
    rosinstall_generator \
    wstool \
    rosinstall \
    empy \
    catkin_tools \
    && pip3 install --upgrade setuptools \
    && pip3 install gym scipy sympy pyserial \
    && deactivate"

# install amdgpu pro components
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    git \
#    pciutils \
#    #apt-utils\
#    wget \
#    build-essential \
#    cmake \
#    libboost-all-dev \
#    && dpkg --add-architecture i386

#ADD ./amdgpu-pro-20.20-1098277-ubuntu-20.04.tar.xz .
#RUN #tar -Jxvf amdgpu-pro-20.20-1098277-ubuntu-20.04.tar.xz && \
	#dpkg --add-architecture i386 && \
#RUN	./amdgpu-pro-20.20-1098277-ubuntu-20.04/amdgpu-pro-install -y

#RUN wget -qO - http://repo.radeon.com/rocm/apt/debian/rocm.gpg.key | apt-key add - && \
#	sh -c 'echo deb [arch=amd64] http://repo.radeon.com/rocm/apt/debian/ focal main > /etc/apt/sources.list.d/rocm.list'

# install plaidml
RUN apt-get update && apt-get install -y --no-install-recommends \
    clinfo x11vnc xvfb

RUN /bin/bash -c "source ~/myenv/bin/activate \ 
    && pip3 install -U plaidml-keras \
    && pip3 uninstall -y enum34 \
    && deactivate"

COPY .plaidml ~/

# install fhtw3dof components
COPY ./src /catkin_ws/src

RUN /bin/bash -c "source ~/myenv/bin/activate \ 
    && pip3 install -e /catkin_ws/src/fhtw3dof/gym-fhtw3dof \
    && deactivate"

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash \
    && cd catkin_ws \
    && catkin_make "

# get application
COPY ./app /app

# xvfb-run roslaunch mairobot MairobotClickAndGo.launch

# https://stackoverflow.com/questions/48561981/activate-python-virtualenv-in-dockerfile#:~:text=virtualenv%20is%20used%20for%20dependency,between%20containers%20and%20between%20applications.

# http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/
