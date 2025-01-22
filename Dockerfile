FROM px4io/px4-dev-ros-noetic AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL [ "/bin/bash", "-o", "pipefail", "-c" ]

ARG UNAME=sim
ARG USE_NVIDIA=1

# Dependencies
RUN sudo apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    python-is-python3 \
    apt-utils \
    byobu \
    fuse \
    git \
    libxext6 \
    libx11-6 \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libfuse-dev \
    libpulse-mainloop-glib0 \
    rapidjson-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \ 
    gstreamer1.0-gl \
    iputils-ping \
    nano \
    wget \
    gz-garden \
    && rm -rf /var/lib/apt/lists/*

# Python deps
RUN sudo pip install PyYAML MAVProxy
RUN sudo pip install --no-cache-dir pyproj
RUN sudo pip install utm

# User
RUN adduser --disabled-password --gecos '' $UNAME
RUN adduser $UNAME sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
ENV HOME=/home/$UNAME
USER $UNAME

# ROS vars
RUN echo "export GZ_VERSION='garden'" >> ~/.bashrc

ENV DISPLAY=:0
RUN echo $DISPLAY

WORKDIR $HOME

# Load packages
ARG PX4_TAG="v1.15.2"
RUN sudo git clone --depth 1 --branch $PX4_TAG --recurse-submodules https://github.com/PX4/PX4-Autopilot.git
RUN git config --global --add safe.directory /home/sim/PX4-Autopilot

# ROS vars
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ~/UDH2025_robotics/catkin_ws/devel/setup.bash --extend" >> ~/.bashrc

# Nvidia GPU vars
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
RUN if [[ -z "${USE_NVIDIA}" ]] ;\
    then printf "export QT_GRAPHICSSYSTEsudo docker imagesM=native" >> /home/${UNAME}/.bashrc ;\
    else echo "Native rendering support disabled" ;\
    fi

# Add terminal commands
RUN echo "alias copy_files='sudo cp /home/sim/UDH2025_robotics/world/wadibirk.sdf /home/sim/UDH2025_robotics/catkin_ws/src/sitl_targets_gazebo-classic.cmake /home/sim/PX4-Autopilot/Tools/simulation/gz/worlds'" >> ~/.bashrc && \
    echo "alias run='/home/sim/UDH2025_robotics/run_multiple.sh 4'" >> ~/.bashrc && \
    echo "alias fly='roslaunch drones_sim px4_sim_recursed.launch n:=4'" >> ~/.bashrc && \
    echo "alias rm_log='rm -rf ~/.ros'" >> ~/.bashrc \
    echo "alias ev='roslaunch mission_evaluation node.launch'" >> ~/.bashrc

