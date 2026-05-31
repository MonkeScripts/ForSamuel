FROM ardusub_sim:humble

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ARG HOME_DIR=/root

RUN apt-get update && apt-get install -y --no-install-recommends \
    atop \
    bash-completion \
    bc \
    cppcheck \
    dmidecode \
    expect \
    git \
    glmark2 \
    gnutls-bin \
    gstreamer1.0-gl \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    libbluetooth-dev \
    libccd-dev \
    libcwiid-dev \
    libegl1 \
    libeigen3-dev \
    libfcl-dev \
    libgflags-dev \
    libgles2-mesa-dev \
    libglvnd0 \
    libglx0 \
    libgoogle-glog-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libimage-exiftool-perl \
    libspnav-dev \
    libusb-dev \
    libx11-6 \
    libxcb-cursor-dev \
    libxcb-xinerama0 \
    libxext6 \
    libxkbcommon-x11-0 \
    libxml2-utils \
    libyaml-cpp-dev \
    libwebsocketpp-dev \
    python3-dbg \
    python3-opencv \
    ros-humble-actuator-msgs \
    ros-humble-ament-cmake-pycodestyle \
    ros-humble-gps-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-joy-linux \
    ros-humble-joy-teleop \
    ros-humble-radar-msgs \
    ros-humble-rqt-graph \
    ros-humble-rqt-image-view \
    ros-humble-rqt-plot \
    ros-humble-rqt-topic \
    ros-humble-vision-msgs \
    ros-humble-image-common \
    ros-humble-navigation2 \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-nav2-bringup \
    ros-humble-nav2-util \
    ros-humble-nav2-common \
    ros-humble-rosx-introspection \
    ros-humble-joint-state-publisher \
    ruby \
    software-properties-common \
    xterm \
    xvfb && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ── tmuxp ─────────────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir tmuxp

# ── ML stack (cherry-picked from src/isaac-ros-docker auv_sim env) ───────────
# `--ignore-installed` is needed because Ubuntu Jammy ships some of these
# packages (e.g. sympy) via apt/distutils which pip 23+ refuses to uninstall —
# pip installs the new version into /usr/local on top instead.
#
# Single pip resolve for the whole ML stack — splitting it across multiple RUN
# blocks defeats pip's solver (it can't see prior --ignore-installed installs)
# and ends up re-installing torch + numpy at incompatible versions in a later
# layer. One big install with every pin in scope avoids that.
#
# Pins:
# - torch==2.7.0+cu128 / torchvision==0.22.0+cu128 — required for NVIDIA
#   Blackwell GPUs (sm_120, e.g. RTX 50-series). The older 2.4.1+cu121 wheels
#   only ship kernels for sm_50..sm_90; on a Blackwell card every CUDA call
#   dies with "no kernel image is available for execution on the device".
# - numpy<2 — ROS Humble's cv_bridge ships a precompiled boost extension
#   linked against the numpy 1.x C ABI. With numpy 2.x installed, every
#   `from cv_bridge import CvBridge` dies with
#   `AttributeError: _ARRAY_API not found`. Torch 2.7 accepts numpy 1.26.x
#   (its install_requires says `numpy`, no upper bound), so we keep numpy
#   on the 1.x line until the whole Humble release ships rebuilt bindings.
# - opencv still pinned <4.12 because 4.12 requires numpy 2.x.
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --ignore-installed \
        --extra-index-url https://download.pytorch.org/whl/cu128 \
        "numpy<2" \
        "opencv-python<4.12" \
        "opencv-contrib-python<4.12" \
        torch==2.7.0+cu128 \
        torchvision==0.22.0+cu128 \
        "onnx<1.18" \
        onnxslim \
        onnxruntime-gpu \
        kornia \
        supervision \
        imutils \
        transforms3d \
        typer \
        rich \
        scikit-learn \
        shapely \
        ultralytics

# Pin setuptools<70 AFTER the ML install (must be last, otherwise the
# --ignore-installed ML resolve re-resolves setuptools to >=70 as a build
# dep of torch/ultralytics, and colcon's --symlink-install breaks with
# `error: option --editable not recognized`).
#
# setuptools 68.2.x and 69.5.x both ship a vendored jaraco that does
# `from backports import tarfile` without declaring backports.tarfile as a
# dep — install it explicitly so the ament_cmake_python build step doesn't
# ImportError.
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --ignore-installed --force-reinstall \
        "setuptools<70" \
        backports.tarfile

WORKDIR ${HOME_DIR}
