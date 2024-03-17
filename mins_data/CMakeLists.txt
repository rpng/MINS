cmake_minimum_required(VERSION 3.3)

# Project name
project(mins_data)

# Find our ROS version!
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if (catkin_FOUND)
    message(STATUS "ROS *1* version found")
    catkin_package()
elseif (ament_cmake_FOUND)
    message(STATUS "ROS *2* version found")
    ament_package()
else ()
    message(STATUS "No ROS versions found, doing nothing...")
endif ()
