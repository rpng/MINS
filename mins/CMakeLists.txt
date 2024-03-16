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

# Find ROS build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2 REQUIRED COMPONENTS rmw_adapters message_filters rosbag2_storage_builtin)
find_package(tf2_ros REQUIRED COMPONENTS tf2_cpp)  # Replaces tf

# Find message packages (geometry_msgs, sensor_msgs, etc.)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_geometry REQUIRED)  # Might require additional setup
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)  # Might require additional setup
find_package(cv_bridge REQUIRED)        # Might require additional setup
find_package(pcl_conversions REQUIRED)

# Other dependencies (if available through ROS 2 packages)
# find_package(pcl_ros2 REQUIRED)  # Might require building ROS 2 wrapper for PCL
find_package(ov_core REQUIRED)   # Might not be available as a ROS 2 package

add_definitions(-DROS_AVAILABLE=2)
ament_export_dependencies(rclcpp rosbag2 tf2_ros std_msgs geometry_msgs sensor_msgs nav_msgs image_geometry visualization_msgs image_transport cv_bridge ov_core pcl_conversions)
ament_export_include_directories(src/)
ament_export_libraries(mins_lib)

list(APPEND ament_libraries
        rclcpp
        rosbag2
        tf2_ros
        std_msgs
        geometry_msgs
        visualization_msgs
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
        ov_core
        pcl_conversions
        image_geometry
        libpointmatcher
)


# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${libpointmatcher_LIBRARIES}
        ${libnabo_LIBRARIES}
        ${PCL_LIBRARIES}
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
ament_target_dependencies(mins_lib ${ament_libraries})
target_link_libraries(mins_lib ${thirdparty_libraries} ${rclcpp_LIBRARIES})

# target_include_directories(mins_lib PUBLIC src/)

target_include_directories(mins_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# install(TARGETS mins_lib
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )
install(TARGETS mins_lib
  EXPORT mins_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# install(DIRECTORY src/
#         DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#         FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#         )
# Install header files
install(DIRECTORY src/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_targets(mins_targets)

# Add packages
##################################################
# Make binary files!
##################################################
add_executable(simulation src/run_simulation.cpp)
target_link_libraries(simulation mins_lib)

# add_executable(bag src/run_bag.cpp)
# target_link_libraries(bag mins_lib)

add_executable(subscribe src/run_subscribe.cpp)
target_link_libraries(subscribe mins_lib)

# Install executables
install(TARGETS subscribe simulation DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)

# add_executable(simulation src/run_simulation.cpp)
# target_link_libraries(simulation mins_lib)
# install(TARGETS simulation
#         ARCHIVE DESTINATION lib/${PROJECT_NAME}
#         LIBRARY DESTINATION lib/${PROJECT_NAME}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

# add_executable(bag src/run_bag.cpp)
# target_link_libraries(bag mins_lib)
# install(TARGETS bag
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION lib/${PROJECT_NAME}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

# add_executable(subscribe src/run_subscribe.cpp)
# target_link_libraries(subscribe mins_lib)
# install(TARGETS subscribe
#         ARCHIVE DESTINATION lib/${PROJECT_NAME}
#         LIBRARY DESTINATION lib/${PROJECT_NAME}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

ament_package()