cmake_minimum_required(VERSION 3.5.1)

# Project name
project(mins_eval)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

# check if we have our python libs files (will search for python3 then python2 installs)
# sudo apt-get install python-matplotlib python-numpy python-dev
# https://cmake.org/cmake/help/latest/module/FindPython.html
# https://stackoverflow.com/a/34580995/7718197
find_package(Python COMPONENTS Interpreter Development)
option(DISABLE_MATPLOTLIB "Disable or enable matplotlib plot scripts in mins_eval" OFF)
if (Python_FOUND AND NOT DISABLE_MATPLOTLIB)
    add_definitions(-DHAVE_PYTHONLIBS=1)
    message(STATUS "PYTHON VERSION: " ${Python_VERSION})
    message(STATUS "PYTHON INCLUDE: " ${Python_INCLUDE_DIRS})
    message(STATUS "PYTHON LIBRARIES: " ${Python_LIBRARIES})
    include_directories(${Python_INCLUDE_DIRS})
    list(APPEND thirdparty_libraries ${Python_LIBRARIES})
endif ()

# We need c++14 for ROS2, thus just require it for everybody
# NOTE: To future self, hope this isn't an issue...
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")

# Find ROS build system
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs nav_msgs sensor_msgs ov_core ov_eval)
find_package(PCL REQUIRED QUIET)

# Describe ROS project
catkin_package(
        CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs sensor_msgs ov_core ov_eval
        INCLUDE_DIRS src/
        LIBRARIES mins_eval_lib
)

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )

##################################################
# Make the shared library
##################################################

add_library(mins_eval_lib SHARED
        src/functions/ErrorPlot.cpp
        src/functions/ResultTrajectory.cpp
        )
target_link_libraries(mins_eval_lib ${thirdparty_libraries})
target_include_directories(mins_eval_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/)
install(TARGETS mins_eval_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

##################################################
# Make binary files!
##################################################

add_executable(plot_consistency src/plot_consistency.cpp)
target_link_libraries(plot_consistency mins_eval_lib ${thirdparty_libraries})
install(TARGETS plot_consistency
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

add_executable(run_comparison src/run_comparison.cpp)
target_link_libraries(run_comparison mins_eval_lib ${thirdparty_libraries})
install(TARGETS run_comparison
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
