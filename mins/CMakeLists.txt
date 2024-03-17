cmake_minimum_required(VERSION 3.5.1)

# Project name
project(mins)

# We need c++14 for ROS2, thus just require it for everybody
# NOTE: To future self, hope this isn't an issue...
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress unnecessary warnings: https://github.com/PointCloudLibrary/pcl/issues/3680
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED QUIET)
endif ()
find_package(Boost REQUIRED QUIET COMPONENTS system filesystem thread date_time program_options chrono)
find_package(PCL REQUIRED QUIET)
find_package(libpointmatcher REQUIRED QUIET)
find_package(libnabo REQUIRED QUIET)

message(STATUS "OPENCV: " ${OpenCV_VERSION} " | BOOST: " ${Boost_VERSION} " | LIBPOINTMATCHER: " ${libpointmatcher_VERSION})
message(STATUS "Eigen3: " ${Eigen3_VERSION} " | PCL: " ${PCL_VERSION} "   | LIBNABO: " ${libnabo_VERSION})

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")

find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if (catkin_FOUND)
    message(STATUS "ROS *1* version found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND)
    message(STATUS "ROS *2* version found, building ROS2.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else ()
    message(STATUS "No ROS versions found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
endif ()