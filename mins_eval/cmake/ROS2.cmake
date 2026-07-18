cmake_minimum_required(VERSION 3.5.1)

find_package(ament_cmake REQUIRED)

# ament_target_dependencies() was dropped from ament_cmake_target_dependencies on newer
# distros (e.g. ROS2 Lyrical) in favor of modern CMake imported targets. Shim it back in
# when missing, falling back to classic ${dep}_INCLUDE_DIRS/_LIBRARIES if no modern target.
if(NOT COMMAND ament_target_dependencies)
  macro(ament_target_dependencies target)
    foreach(_dep ${ARGN})
      if(TARGET ${_dep}::${_dep})
        target_link_libraries(${target} ${_dep}::${_dep})
      else()
        if(${_dep}_INCLUDE_DIRS)
          target_include_directories(${target} PUBLIC ${${_dep}_INCLUDE_DIRS})
        endif()
        if(${_dep}_LIBRARIES)
          target_link_libraries(${target} ${${_dep}_LIBRARIES})
        endif()
        if(${_dep}_DEFINITIONS)
          target_compile_definitions(${target} PUBLIC ${${_dep}_DEFINITIONS})
        endif()
      endif()
    endforeach()
  endmacro()
endif()

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
        ${PYTHON_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
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