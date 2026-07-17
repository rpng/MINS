cmake_minimum_required(VERSION 3.5.1)

# Find ROS1 (catkin). QUIET (not REQUIRED) so this same file can also configure a
# ROS-free build of the core library when catkin is not present.
find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs image_geometry visualization_msgs image_transport cv_bridge ov_core pcl_ros)
option(ENABLE_ROS "Build the ROS integration and nodes when ROS is found" ON)

# When ROS is absent, ov_core (normally a catkin package) is pulled in as a subproject.
# Do it here, before we populate LIBRARY_SOURCES: add_subdirectory shares this scope's
# variables with the child, and ov_core also appends to LIBRARY_SOURCES.
if (NOT (catkin_FOUND AND ENABLE_ROS))
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/open_vins/ov_core ${CMAKE_BINARY_DIR}/ov_core)
endif ()

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${OpenCV_LIBRARIES}
        )

# Core (ROS-free) library sources
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
        src/utils/PackagePath.cpp
        src/utils/Print_Logger.cpp
        src/utils/Jabdongsani.cpp
        src/sim/Simulator.cpp
        src/sim/ConstBsplineSE3.cpp
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

if (catkin_FOUND AND ENABLE_ROS)
    message(STATUS "MINS: building WITH ROS1")
    add_definitions(-DROS_AVAILABLE=1)

    # Add catkin packages
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs image_geometry visualization_msgs image_transport cv_bridge ov_core pcl_ros
            INCLUDE_DIRS src/
            LIBRARIES mins_lib
    )
    include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
    list(APPEND thirdparty_libraries ${catkin_LIBRARIES})

    # ROS integration lives at the boundary - compiled in only when we have ROS
    list(APPEND LIBRARY_SOURCES
            src/core/ROSPublisher.cpp
            src/core/ROSSubscriber.cpp
            src/core/ROSHelper.cpp
            src/sim/SimVisualizer.cpp
            )
else ()
    message(WARNING "MINS: building WITHOUT ROS - core library + headless CLI only")
    add_definitions(-DROS_AVAILABLE=0)
    include(GNUInstallDirs)
    set(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_LIBDIR}")
    set(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_BINDIR}")
    set(CATKIN_GLOBAL_INCLUDE_DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mins/")

    # ov_core was pulled in above (before LIBRARY_SOURCES); just link + find PCL here.
    # (In a ROS build PCL comes via catkin/pcl_ros; standalone we link it ourselves.)
    # Only the components we actually use: point types + transforms (common) and VoxelGrid
    # (filters). Asking for all of PCL drags in pcl_visualization -> VTK, which we never use.
    find_package(PCL REQUIRED COMPONENTS common filters)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    list(APPEND thirdparty_libraries ov_core_lib ${PCL_LIBRARIES})
endif ()

##################################################
# Make the shared library
##################################################

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

if (catkin_FOUND AND ENABLE_ROS)

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

else ()

    # Headless CLI: runs the simulator through the estimator with no ROS, writes
    # results to file. This is the ROS-free runnable target.
    add_executable(mins_cli src/run_simulation_cli.cpp)
    target_link_libraries(mins_cli mins_lib)
    install(TARGETS mins_cli
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

endif ()
