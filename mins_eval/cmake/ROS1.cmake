cmake_minimum_required(VERSION 3.3)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs nav_msgs sensor_msgs ov_core ov_eval)
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
