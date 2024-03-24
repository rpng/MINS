cmake_minimum_required(VERSION 3.3)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ov_core REQUIRED)   # Might not be available as a ROS 2 package
find_package(ov_eval REQUIRED)   # Might not be available as a ROS 2 package
find_package(mins REQUIRED)

add_definitions(-DROS_AVAILABLE=2)
# Describe ROS project
ament_export_dependencies(roscpp rospy geometry_msgs nav_msgs sensor_msgs ov_core ov_eval mins)
ament_export_libraries(mins_eval_lib)

list(APPEND ament_libraries
        rclcpp
        rosbag2
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        ov_core
        ov_eval
        mins
)

# Include our header files
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${libpointmatcher_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        )

##################################################
# Make the shared library
##################################################

add_library(mins_eval_lib SHARED
        src/functions/ErrorPlot.cpp
        src/functions/ResultTrajectory.cpp
        )
target_link_libraries(mins_eval_lib ${thirdparty_libraries})
ament_target_dependencies(mins_eval_lib ${ament_libraries})
#target_include_directories(mins_eval_lib PUBLIC ${thirdparty_libraries} ${rclcpp_LIBRARIES})

install(TARGETS mins_eval_lib
  EXPORT mins_eval_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY src/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_targets(mins_eval_targets)

##################################################
# Make binary files!
##################################################

add_executable(plot_consistency src/plot_consistency.cpp)
target_link_libraries(plot_consistency mins_eval_lib)
install(TARGETS plot_consistency DESTINATION lib/${PROJECT_NAME})

add_executable(run_comparison src/run_comparison.cpp)
target_link_libraries(run_comparison mins_eval_lib)
install(TARGETS run_comparison DESTINATION lib/${PROJECT_NAME})

ament_package()