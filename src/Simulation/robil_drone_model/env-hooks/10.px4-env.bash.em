#!/bin/bash

export PX4_PATH=$(rospack find px4 > /dev/null 2&>1)

if [ -z "${PX4_PATH}" ]
then
    echo "Px4 firmware package not found!"
else
    . ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/posix_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_PATH}
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX4_PATH}/Tools/sitl_gazebo
    export ROBIL_PX4_FOUND=true
fi