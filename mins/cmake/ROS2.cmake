cmake_minimum_required(VERSION 3.3)

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
find_package(std_srvs REQUIRED)
find_package(image_geometry REQUIRED)  # Might require additional setup
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)  # Might require additional setup
find_package(cv_bridge REQUIRED)        # Might require additional setup
find_package(pcl_conversions REQUIRED)

# Other dependencies (if available through ROS 2 packages)
# find_package(pcl_ros2 REQUIRED)  # Might require building ROS 2 wrapper for PCL
find_package(ov_core REQUIRED)   # Might not be available as a ROS 2 package

add_definitions(-DROS_AVAILABLE=2)
ament_export_dependencies(rclcpp rosbag2 tf2_ros std_msgs geometry_msgs sensor_msgs nav_msgs std_srvs image_geometry visualization_msgs image_transport cv_bridge ov_core pcl_conversions)
# ament_export_include_directories(src/)
ament_export_libraries(mins_lib)

list(APPEND ament_libraries
        rclcpp
        rosbag2
        tf2_ros
        std_msgs
        std_srvs
        geometry_msgs
        visualization_msgs
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
        ov_core
        pcl_conversions
        image_geometry
)


# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${libpointmatcher_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${libnabo_LIBRARIES}
        ${PCL_LIBRARIES}
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
        src/core/ROS2Publisher.cpp
        src/core/ROS2Subscriber.cpp
        src/core/ROS2Helper.cpp
        src/sim/Simulator.cpp
        src/sim/ConstBsplineSE3.cpp
        src/sim/Sim2Visualizer.cpp
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

install(TARGETS mins_lib
  EXPORT mins_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY
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

add_executable(subscribe src/run_subscribe.cpp)
target_link_libraries(subscribe mins_lib)

# Install executables
install(TARGETS subscribe simulation DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)

ament_package()