#! /bin/bash

CAMERA_ROS_VERSION='0.5.0'
XRCE_DSS_VERSION='v3.0.1'

function git_clone_pull() {
    cd ${PPWD}/src
    R=${1}
    D=${2}
    V=${3}
    M=${4}

    if [[ ! -e ${D} ]]; then
        git clone ${R} || fail "unable to clone camera ros"
    fi

    cd ${PPWD}/src/${D}
    git checkout ${M}
    git branch -d ${V}
    git fetch && git pull origin ${M}
    git checkout -b ${V} tags/${V}

}

PPWD=$(pwd)
source ${PPWD}/scripts/common.bash

if [[ ! -e ${PPWD}/src ]]; then
    mkdir -p ${PPWD}/src
fi

cd ${PPWD}/src
echo "INFO: entered ${PPWD}/src"
echo "INFO: refreshing camera_ros:${CAMERA_ROS_VERSION}"
git_clone_pull 'https://github.com/christianrauch/camera_ros.git' camera_ros ${CAMERA_ROS_VERSION} main

echo "INFO: refreshing Micro-XRCE-DDS-Agent:${XRCE_DSS_VERSION}"
git_clone_pull 'https://github.com/eProsima/Micro-XRCE-DDS-Agent.git' Micro-XRCE-DDS-Agent ${XRCE_DSS_VERSION} master
# rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera

exit 0