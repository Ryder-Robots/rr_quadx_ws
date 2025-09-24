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

# XRCE DDS Server and Client version
XRCE_DDS_VERSION='v3.0.1'

LIB_CAMERA_INSTALL=0
#=======================================

# Update .bashrc
if [[ -e ${HOME}/.bashrc.ros2.back  ]]; then
    cp ${HOME}/.bashrc.ros2.back ${HOME}/.bashrc
else 
    cp ${HOME}/bashrc ${HOME}/.bashrc.ros2.back
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