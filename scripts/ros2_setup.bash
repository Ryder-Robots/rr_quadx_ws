#! /bin/bash

# Setup script for ROS2,  this installation assumes that Pi has version as described in
# in https://www.instructables.com/Install-ROS-on-Your-Raspberry-Pi-5-With-ROCKO/. 
#
# This script should only be ran to change ROS2 distro,  and not for general compiling.
#
# Installation procedure aligns with https://docs.ros.org/en/${ROS_DISTRO}/Installation/Ubuntu-Install-Debs.html
# However assumes that locale supports UTF-8

#=======================================
# CONSTANTS - The following is a list of constants used by this script.

ROS_DOMAIN_ID=101

# ';' sepearated lists of which hosts are allowed to access robot code
ROS_STATIC_PEERS=192.168.1.15

# ROS Discovery range can be one of the following:
#.  SUBNET, LOCALHOST, OFF, SYSTEM_DEFAULT
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# ROS2 distro to include.
ROS_DISTRO=kilted

#TODO: this belongs in compile script.
# # XRCE DDS Server and Client version
# XRCE_DDS_VERSION='v3.0.1'

# LIB_CAMERA_INSTALL=0
#=======================================

PPWD=$(pwd)
source ${PPWD}/scripts/common.bash


# Wrapper for installing packages
function aptinstall(){
    pkg=${1}
    sudo apt-get install -y  ${pkg}
    if [[ $? -ne 0 ]]; then
        echo "ERROR: unable to install package: ${pkg}"
        exit 1
    fi
}

# update package cahes
function update_pkg_cache() {
  echo "INFO: attempting to upgrade packages"
  sudo apt update && apt-get upgrade -y 
}



echo "INFO: start installing ROS2"
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb || fail "could not add ros2 apt sources"
update_pkg_cache
aptinstall ros-${ROS_DISTRO}-ros-base
aptinstall ros-dev-tools

echo "INFO: installing controller software"
aptinstall ros-${ROS_DISTRO}-ros2-control
aptinstall ros-${ROS_DISTRO}-ros2-controllers
aptinstall ros-${ROS_DISTRO}-foxglove-bridge
aptinstall ros-${ROS_DISTRO}-camera-calibration

echo "INFO: installing ROS2 dependencies"
aptinstall python3-rosdep
rosdep init
rosdep update

# Update .bashrc
if [[ -e ${HOME}/.bashrc.ros2.back  ]]; then
    cp ${HOME}/.bashrc.ros2.back ${HOME}/.bashrc
else 
    cp ${HOME}/.bashrc ${HOME}/.bashrc.ros2.back
fi

# Updating Bash RC
echo "INFO: Updating bashrc ROS initlization"
grep '/opt/ros/${ROS_DISTRO}/setup.bash' ${HOME}/.bashrc || echo "/opt/ros/${ROS_DISTRO}/setup.bash" "source /opt/ros/${ROS_DISTRO}/setup.bash" 
grep ROS_DOMAIN_ID ${HOME}/.bashrc || echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> ${HOME}/.bashrc
grep ROS_AUTOMATIC_DISCOVERY_RANGE ${HOME}/.bashrc || echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE}" >> ${HOME}/.bashrc
grep ROS_STATIC_PEERS ${HOME}/.bashrc || echo  "export ROS_STATIC_PEERS='${ROS_STATIC_PEERS}'"  >> ${HOME}/.bashrc
grep ROS_DISTRO ${HOME}/.bashrc || echo  "export ROS_DISTRO='${ROS_DISTRO}'"  >> ${HOME}/.bashrc
echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ${HOME}/.bashrc

echo "INFO: sourcing .bashrc with ROS2 updates"
source ${HOME}/.bashrc

exit 0