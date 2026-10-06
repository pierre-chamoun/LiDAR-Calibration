FROM osrf/ros:jazzy-desktop

ARG UID=1000
ARG GID=1000
ARG USERNAME=user

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Dev tools
    nano \
    vim \
    git \
    cmake \
    build-essential \
    software-properties-common \
    # Ceres Solver and dependencies
    libceres-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    # ROS2 extras
    libomp-dev \
    ros-jazzy-sophus \
    ros-jazzy-pcl-ros \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install GTSAM from source (PPA does not support Ubuntu Noble)
RUN git clone --branch 4.2.0 --depth 1 https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cmake -S /tmp/gtsam -B /tmp/gtsam/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_UNSTABLE=ON \
    && cmake --build /tmp/gtsam/build --parallel $(nproc) \
    && cmake --install /tmp/gtsam/build \
    && ldconfig \
    && rm -rf /tmp/gtsam

# Build and install fast_gicp from source
RUN git clone https://github.com/SMRT-AIST/fast_gicp.git /tmp/fast_gicp \
    && cmake -S /tmp/fast_gicp -B /tmp/fast_gicp/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_apps=OFF \
        -DBUILD_test=OFF \
    && cmake --build /tmp/fast_gicp/build --parallel $(nproc) \
    && cp -r /tmp/fast_gicp/include/fast_gicp /usr/local/include/ \
    && find /tmp/fast_gicp/build \( -name "*.a" -o -name "*.so*" \) -exec cp -P {} /usr/local/lib/ \; \
    && ldconfig \
    && rm -rf /tmp/fast_gicp

# Rename the existing ubuntu user to match the host username
RUN groupmod -n ${USERNAME} ubuntu \
    && usermod -l ${USERNAME} -d /home/${USERNAME} -m ubuntu \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

# Workspace
RUN mkdir -p /home/${USERNAME}/ros_ws/src \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/ros_ws
WORKDIR /home/${USERNAME}/ros_ws

USER ${USERNAME}

CMD ["/bin/bash"]
