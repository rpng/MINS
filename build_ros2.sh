#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build --paths src/MINS/thirdparty/*
source install/setup.bash
colcon build --paths src/MINS/thirdparty/open_vins/*
source install/setup.bash
colcon build --paths src/MINS/{mins,mins_data}
source install/setup.bash
colcon build --paths src/MINS/mins_eval
