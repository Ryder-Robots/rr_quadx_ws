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
ROS_DISTRO=jazzy

UBUNTU_CODENAME=$(lsb_release -c | cut -d : -f 2 | xargs)

#TODO: this belongs in compile script.
# # XRCE DDS Server and Client version
# XRCE_DDS_VERSION='v3.0.1'

# LIB_CAMERA_INSTALL=0
#=======================================

PPWD=$(pwd)
source ${PPWD}/scripts/common.bash

# update package cahes
function update_pkg_cache() {
  echo "INFO: attempting to upgrade packages"
  sudo apt update && sudo apt-get upgrade -y 
}

function fail() {
   msg=${1}
   echo "ERROR: unable to install: ${msg}"
   exit 1
}

echo "INFO: start installing ROS2"
echo "INFO: Enable required repositories"
# aptinstall software-properties-common

# Run this command manually
#sudo add-apt-repository universe | fail "could not add ros repositories"

echo "INFO: installing ros2-apt source packages"
aptinstall curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "INFO: installing ROS2 development tools"
aptinstall ros-dev-tools

echo "INFO: install ros $ROS_DISTRO"
aptinstall ros-$ROS_DISTRO-ros-base


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
echo 'source ${HOME}/rr_quadx_ws/setup.bash' >> ${HOME}/.bashrc

echo "INFO: sourcing .bashrc with ROS2 updates"
source ${HOME}/.bashrc

exit 0