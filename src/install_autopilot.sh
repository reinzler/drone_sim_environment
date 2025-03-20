#!/bin/bash

# 1. Создание папки drone_autopilot
echo "Создание папки drone_autopilot..."
mkdir -p ~/drone_autopilot
cd ~/drone_autopilot || exit

# 2. Обновление системы и установка базовых зависимостей
echo "Обновление системы и установка зависимостей..."
sudo apt update
sudo apt install -y git cmake build-essential python3-pip python3-venv wget curl unzip \
    libopencv-dev python3-opencv ros-$ROS_DISTRO-desktop-full ros-$ROS_DISTRO-mavros*
sudo pip install --user -U empy==3.3.4 pyros-genmsg setuptools

# 3. Установка Fast DDS (если необходимо)
echo "Micro-XRCE-DDS-Agent"
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
cd ../..

# 4. Клонирование PX4-Autopilot
echo "Клонирование PX4-Autopilot..."
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot && make px4_sitl -j$(nproc) || exit

# 5. Клонирование px4msg, px4ros_com
echo "Клонирование px4msg, px4ros_com"
mkdir px4_bridge $$ cd px4_bridge
mkdir src $$ cd src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
colcon build
source install/local_setup.bash  # <-- Это гарантирует, что новые пакеты доступны
echo "source $(pwd)/install/local_setup.bash" >> ~/.bashrc
cd -

# 6. Установка PX4 keyboard teleop
echo "Клонирование px4_keyboard_teleop_control"
git clone https://github.com/DCVAM/px4_keyboard_teleop_control.git
cd px4_keyboard_teleop_control
colcon build
source install/local_setup.bash  # <-- Это гарантирует, что новые пакеты доступны
echo "source $(pwd)/install/local_setup.bash" >> ~/.bashrc
cd -

# 7. Клонирование Pangolin
echo "Клонирование Pangolin..."
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
./scripts/install_prerequisites.sh --dry-run recommended # Check what recommended softwares needs to be installed
./scripts/install_prerequisites.sh recommended # Install recommended dependencies
cmake -B build
cmake --build build -j$(nproc)
sudo cmake --install build
cd ../..

# 8. Проверка и настройка Настройка LD_LIBRARY_PATH
echo "Проверка и настройка LD_LIBRARY_PATH..."
if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
    echo "LD_LIBRARY_PATH обновлен."
else
    echo "LD_LIBRARY_PATH уже содержит /usr/local/lib."
fi

# Добавление LD_LIBRARY_PATH в .bashrc
if ! grep -q "export LD_LIBRARY_PATH=/usr/local/lib" ~/.bashrc; then
    echo "Добавление LD_LIBRARY_PATH в .bashrc..."
    echo 'if [[ ":$LD_LIBRARY_PATH:" != *":/usr/local/lib:"* ]]; then' >> ~/.bashrc
    echo '    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
    echo 'fi' >> ~/.bashrc
    source ~/.bashrc
else
    echo "LD_LIBRARY_PATH уже настроен в .bashrc."
fi

sudo ldconfig
source ~/.bashrc

# 9. Клонирование ORB-SLAM_3_ROS2
echo "Клонирование ORB-SLAM_3_ROS2..."
mkdir -p ~/ros2_test/src
git clone https://github.com/reinzler/ros2_orb_slam3.git
cd ..
rosdep install -r --from-paths src --ignore-src -y --rosdistro humble
source /opt/ros/humble/setup.bash
#colcon build --symlink-install -j$(nproc)
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)


# 10. Завершение
echo "Установка завершена! Все компоненты находятся в папке ~/drone_autopilot."
echo "Чтобы запустить PX4 SITL, выполните:"
echo "cd ~/drone_autopilot/PX4-Autopilot && make px4_sitl gz_x500_gimbal_windy"
echo "Чтобы запустить XRCE-DDS-Agent, выполните:"
echo "MicroXRCEAgent udp4 -p 8888"
echo "Чтобы проверить установку и работу ORB_SLAM3_ROS2:"
echo "cd ~/drone_autopilot/ros2_test/ && source ./install/setup.bash && ros2 run ros2_orb_slam3 mono_node_cpp --ros-args -p node_name_arg:=mono_slam_cpp"
echo "В другом  терминале:"
echo "cd ~/drone_autopilot/ros2_test/ && source ./install/setup.bash && ros2 run ros2_orb_slam3 mono_driver_node.py --ros-args -p settings_name:=EuRoC -p image_seq:=sample_euroc_MH05"
