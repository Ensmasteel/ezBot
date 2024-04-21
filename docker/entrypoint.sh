#!/bin/bash
set -e

id -u ros &>/dev/null || adduser --quiet --disabled-password --gecos '' --uid ${UID:=1000} --uid ${GID:=1000} ros

source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash

echo "source /home/ezbot/ezbot/install/setup.bash"
source /ezbot/ezbot/install/setup.bash


exec "$@"