#!/bin/bash
set -e

# MINS repo root (dir holding this script) and the colcon workspace root above src/.
MINS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${MINS_DIR}/../.."

source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --paths "${MINS_DIR}"/thirdparty/*
source install/setup.bash
colcon build --paths "${MINS_DIR}"/thirdparty/open_vins/*
source install/setup.bash
colcon build --paths "${MINS_DIR}"/mins "${MINS_DIR}"/mins_data
source install/setup.bash
colcon build --paths "${MINS_DIR}"/mins_eval
