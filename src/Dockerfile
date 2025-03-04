# Базовый образ с ROS 2 Humble
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
# Добавление репозитория
RUN apt update && apt install -y software-properties-common
RUN add-apt-repository universe && apt update
RUN apt update && apt install -y python3-pip
RUN pip3 install catkin_pkg


# Установка зависимостей
RUN apt-get update && apt-get install -y \
    git wget nano curl build-essential cmake python3-pip \
    python3-colcon-common-extensions python3-vcstool \
    libopencv-dev libeigen3-dev libboost-all-dev \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-rclcpp ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Создание рабочей директории
WORKDIR /root/drone_autopilot

# Установка PX4 Autopilot
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    cd PX4-Autopilot && make px4_sitl -j$(nproc)

# Установка зависимостей для Pangolin
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
    cmake libglew-dev libegl1-mesa-dev libxkbcommon-dev \
    libwayland-dev libxi-dev libxxf86vm-dev libunwind-14-dev \
    wayland-protocols libgl1-mesa-dev libc++-14-dev libc++-dev \
    libc++1-14 libc++abi1-14 libepoxy-dev

# Установка Pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    bash ./scripts/install_prerequisites.sh --dry-run recommended && \
    bash ./scripts/install_prerequisites.sh recommended && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Установка ORB-SLAM3 (ROS 2)
RUN echo -e "\e[32m=== Устанавливаем ORB-SLAM3 (ROS 2) ===\e[0m" && \
    mkdir -p /root/drone_autopilot/ros2_orb_slam3_ws/src && \
    cd /root/drone_autopilot/ros2_orb_slam3_ws/src && \
    git clone --recursive https://github.com/reinzler/ros2_orb_slam3.git && \
    cd .. && \
    rosdep update && \
    rosdep install -r --from-paths src --ignore-src -y --rosdistro humble && \
    bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers $(nproc)"


# Установка Micro XRCE-DDS Agent
# Установка Micro XRCE-DDS Agent
RUN git clone --recursive https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/root/drone_autopilot/Micro-XRCE-DDS-Agent .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig /usr/local/lib/


# Установка среды запуска
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/drone_autopilot/orb_slam3/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]

