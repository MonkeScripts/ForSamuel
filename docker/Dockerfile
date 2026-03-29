FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

ENV TZ=Etc/UTC
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
    echo $TZ > /etc/timezone && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime

# ── System dependencies ──────────────────────────────────────────────────────
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release

# Setup Gazebo/OSRF repository
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    atop \
    bash-completion \
    bc \
    build-essential \
    ca-certificates \
    cmake \
    cppcheck \
    dmidecode \
    expect \
    gdb \
    git \
    glmark2 \
    gnutls-bin \
    gstreamer1.0-gl \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gz-harmonic \
    python3-gz-msgs10 \
    python3-gz-transport13 \
    iputils-ping \
    libbluetooth-dev \
    libboost-all-dev \
    libccd-dev \
    libcwiid-dev \
    libegl1 \
    libeigen3-dev \
    libfcl-dev \
    libgflags-dev \
    libgles2-mesa-dev \
    libgl1 \
    libglvnd0 \
    libglx0 \
    libgoogle-glog-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgz-sim8-dev \
    libimage-exiftool-perl \
    libopencv-dev \
    libspnav-dev \
    libusb-dev \
    libx11-6 \
    libxcb-cursor-dev \
    libxcb-xinerama0 \
    libxext6 \
    libxkbcommon-x11-0 \
    libxml2-utils \
    libwebsocketpp-dev \
    lsb-release \
    net-tools \
    pkg-config \
    protobuf-compiler \
    python3-dbg \
    python3-empy \
    python3-pip \
    python3-rospkg \
    python3-sdformat13 \
    python3-setuptools \
    python3-vcstool \
    python3-venv \
    rapidjson-dev \
    ros-humble-actuator-msgs \
    ros-humble-ament-cmake-pycodestyle \
    ros-humble-desktop \
    ros-humble-gps-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-joy-linux \
    ros-humble-joy-teleop \
    ros-humble-mavros-msgs \
    ros-humble-radar-msgs \
    ros-humble-ros-gzharmonic \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    ros-humble-rqt-plot \
    ros-humble-rqt-topic \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-vision-msgs \
    ros-humble-xacro \
    ros-humble-image-common \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-navigation2 \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav2-bringup \
    ros-humble-nav2-util \
    ros-humble-nav2-common \
    ros-humble-rosx-introspection \
    ros-humble-sdformat-urdf \
    ros-humble-joint-state-publisher \
    ruby \
    software-properties-common \
    sudo \
    tmux \
    vim \
    wget \
    xterm \
    xvfb && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

ARG HOME_DIR=/root
ARG ARDUSUB_DIR=${HOME_DIR}/auv

# ── ArduSub SITL ─────────────────────────────────────────────────────────────
WORKDIR ${ARDUSUB_DIR}
RUN git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules --depth 1

WORKDIR ${ARDUSUB_DIR}/ardupilot
ENV SKIP_AP_EXT_ENV=1 \
    SKIP_AP_GRAPHIC_ENV=1 \
    SKIP_AP_COV_ENV=1 \
    SKIP_AP_GIT_CHECK=1
RUN sed -i 's/if \[ \$EUID == 0 \]/if [ 1 == 0 ]/g' \
        Tools/environment_install/install-prereqs-ubuntu.sh && \
    sed -i 's/PIP_USER_ARGUMENT="--user"/PIP_USER_ARGUMENT=""/g' \
        Tools/environment_install/install-prereqs-ubuntu.sh && \
    sed -i 's/sudo usermod -a -G dialout \$USER/true/g' \
        Tools/environment_install/install-prereqs-ubuntu.sh
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y
RUN modules/waf/waf-light configure --board sitl && \
    modules/waf/waf-light build --target bin/ardusub

# ── ardupilot_gazebo plugin (Gazebo Harmonic) ────────────────────────────────
WORKDIR ${ARDUSUB_DIR}
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git

ENV GZ_VERSION=harmonic
RUN cd ardupilot_gazebo && \
    git checkout 50e62a55242e2a35f3fed13920faca02818a47e3 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j$(nproc)

# ── GeographicLib datasets (required by MAVROS) ──────────────────────────────
WORKDIR ${HOME_DIR}
RUN wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# ── Environment ──────────────────────────────────────────────────────────────
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=${ARDUSUB_DIR}/ardupilot_gazebo/build
ENV GZ_VERSION=harmonic
ENV PATH="${ARDUSUB_DIR}/ardupilot/build/sitl/bin:${ARDUSUB_DIR}/ardupilot/Tools/autotest:/root/.local/bin:${PATH}"

# ── tmuxp ─────────────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir tmuxp

WORKDIR ${HOME_DIR}
