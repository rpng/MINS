cmake_minimum_required(VERSION 3.5.1)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs image_geometry visualization_msgs image_transport cv_bridge ov_core pcl_ros)

add_definitions(-DROS_AVAILABLE=1)

# Add catkin packages
catkin_package(
        CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs image_geometry visualization_msgs image_transport cv_bridge ov_core pcl_ros
        INCLUDE_DIRS src/
        LIBRARIES mins_lib
)

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${libpointmatcher_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${libpointmatcher_LIBRARIES}
        )

##################################################
# Make the shared library
##################################################

list(APPEND LIBRARY_SOURCES
        src/options/Options.cpp
        src/options/OptionsCamera.cpp
        src/options/OptionsEstimator.cpp
        src/options/OptionsGPS.cpp
        src/options/OptionsIMU.cpp
        src/options/OptionsInit.cpp
        src/options/OptionsLidar.cpp
        src/options/OptionsSimulation.cpp
        src/options/OptionsSystem.cpp
        src/options/OptionsVicon.cpp
        src/options/OptionsWheel.cpp
        src/core/ROSPublisher.cpp
        src/core/ROSSubscriber.cpp
        src/core/ROSHelper.cpp
        src/sim/Simulator.cpp
        src/sim/ConstBsplineSE3.cpp
        src/sim/SimVisualizer.cpp
        src/utils/Print_Logger.cpp
        src/utils/Jabdongsani.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/SystemManager.cpp
        src/update/cam/CamTypes.cpp
        src/update/cam/CamHelper.cpp
        src/update/cam/UpdaterCamera.cpp
        src/update/vicon/UpdaterVicon.cpp
        src/update/gps/UpdaterGPS.cpp
        src/update/wheel/UpdaterWheel.cpp
        src/update/lidar/ikd_Tree.cpp
        src/update/lidar/UpdaterLidar.cpp
        src/update/lidar/LidarHelper.cpp
        src/update/lidar/LidarTypes.cpp
        src/update/UpdaterStatistics.cpp
        src/init/Initializer.cpp
        src/init/imu/I_Initializer.cpp
        src/init/imu_wheel/IW_Initializer.cpp
        )


file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(mins_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_link_libraries(mins_lib ${thirdparty_libraries})
target_include_directories(mins_lib PUBLIC src/)
install(TARGETS mins_lib
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

add_executable(simulation src/run_simulation.cpp)
target_link_libraries(simulation mins_lib)
install(TARGETS simulation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

add_executable(bag src/run_bag.cpp)
target_link_libraries(bag mins_lib)
install(TARGETS bag
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

add_executable(subscribe src/run_subscribe.cpp)
target_link_libraries(subscribe mins_lib)
install(TARGETS subscribe
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )