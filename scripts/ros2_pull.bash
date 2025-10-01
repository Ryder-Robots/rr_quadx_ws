#! /bin/bash

PPWD=$(pwd)
source ${PPWD}/scripts/common.bash

echo "INFO: Install Utilities"
aptinstall ros-jazzy-image-transport-plugins 
aptinstall v4l-utils

echo "INFO: installing libcamera binary for Pi 5"
aptinstall ppa:marco-sonic/rasppios
aptinstall libcamera-tools 
aptisntall rpicam-apps-lite 
aptinstall python3-picamera2

U=${USER}
sudo adduser $U video


sudo apt -y install python3-colcon-meson
mdkir -p ${PPWD}/src
cd ${PPWD}/src
git clone https://github.com/christianrauch/camera_ros.git

cd ${PPWD}
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
sudo apt remove ros-$ROS_DISTRO-ros-camera
cd ${PPWD}/src
git_clone_pull 'https://github.com/christianrauch/camera_ros.git' camera_ros ${CAMERA_ROS_VERSION} main
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
colcon build --event-handlers=console_direct+


# CAMERA_ROS_VERSION='0.5.0'
# XRCE_DSS_VERSION='v3.0.1'

# function git_clone_pull() {
#     cd ${PPWD}/src
#     R=${1}
#     D=${2}
#     V=${3}
#     M=${4}

#     if [[ ! -e ${D} ]]; then
#         git clone ${R} || fail "unable to clone camera ros"
#     fi

#     cd ${PPWD}/src/${D}
#     git checkout ${M}
#     git branch -d ${V}
#     git fetch && git pull origin ${M}
#     git checkout -b ${V} tags/${V}

# }



# if [[ ! -e ${PPWD}/src ]]; then
#     mkdir -p ${PPWD}/src
# fi

# cd ${PPWD}/src
# echo "INFO: entered ${PPWD}/src"
# echo "INFO: refreshing camera_ros:${CAMERA_ROS_VERSION}"
# git_clone_pull 'https://github.com/christianrauch/camera_ros.git' camera_ros ${CAMERA_ROS_VERSION} main

# echo "INFO: refreshing Micro-XRCE-DDS-Agent:${XRCE_DSS_VERSION}"
# git_clone_pull 'https://github.com/eProsima/Micro-XRCE-DDS-Agent.git' Micro-XRCE-DDS-Agent ${XRCE_DSS_VERSION} master
# rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera

exit 0