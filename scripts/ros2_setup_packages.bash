#! /bin/bash

PPWD=$(pwd)
source ${PPWD}/scripts/common.bash

echo "INFO: Install Utilities"
aptinstall ros-jazzy-image-transport-plugins 
aptinstall v4l-utils

echo "INFO: installing libcamera binary for Pi 5"
sudo add-apt-repository ppa:marco-sonic/rasppios
sudo apt update && $ sudo apt upgrade
aptinstall libcamera-tools 
aptinstall rpicam-apps-lite 
aptinstall python3-picamera2

U=${USER}
sudo adduser $U video

aptinstall python3-colcon-meson
mdkir -p ${PPWD}/src

# referenced from https://github.com/ARLunan/Raspberry-Pi-Camera-ROS/blob/main/Documents/RPCamera-Installation-RP5.md
echo "INFO: installing camera_ros"
if [[ ! -d  ${PPWD}/src/camera_ros ]]; then
    cd ${PPWD}/src
    git clone -b 0.5.0 https://github.com/christianrauch/camera_ros.git
else
    cd ${PPWD}/src/camera_ros
    git pull
fi

cd ${PPWD}
source /opt/ros/$ROS_DISTRO/setup.bash
dpkg -l ros-$ROS_DISTRO-ros-camera
if [[ $? -eq 0 ]]; then
    sudo apt remove ros-$ROS_DISTRO-ros-camera
fi
cd ${PPWD}/src

echo "INFO: setting up Micro-XRCE-DDS-Agent"
source /opt/ros/ROS_DISTRO/setup.bash
if [[ ! -d  ${PPWD}/src/Micro-XRCE-DDS-Agent ]]; then
    cd ${PPWD}/src
    git clone -b v2.4.3  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
else
    cd ${PPWD}/src/Micro-XRCE-DDS-Agent
    git pull
fi


rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
colcon build --event-handlers=console_direct+

exit 0