cmake_minimum_required(VERSION 3.5.1)

# Project name
project(mins)

# We need c++17 for std::filesystem
# NOTE: To future self, hope this isn't an issue...
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress unnecessary warnings: https://github.com/PointCloudLibrary/pcl/issues/3680
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

if (MSVC)
    # M_PI etc need _USE_MATH_DEFINES; NOMINMAX stops windows.h clobbering std::min/max; /bigobj for heavy templates.
    # Export all symbols so the SHARED libs behave like they do on linux/mac (MSVC hides them otherwise);
    # set before add_subdirectory(ov_core) so ov_core_lib inherits it too.
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES)
    # /FIcassert force-includes <cassert> everywhere: ov_core uses assert() but relies on it
    # being pulled in transitively, which gcc/clang do and msvc does not.
    add_compile_options(/O2 /EHsc /bigobj /FIcassert)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
else ()
    # Enable compile optimizations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    # Enable debug flags (use if you want to debug in gdb)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")
endif ()

# Coverage instrumentation (opt-in: cmake -DENABLE_COVERAGE=ON -DBUILD_TESTS=ON)
option(ENABLE_COVERAGE "Instrument for gcov coverage. Forces -O0, so never use for a real build." OFF)
if (ENABLE_COVERAGE)
    if (MSVC)
        message(FATAL_ERROR "ENABLE_COVERAGE needs a gcc or clang toolchain")
    endif ()
    # The -O3 set above would inline away the very lines we are trying to measure.
    string(REPLACE "-O3" "-O0" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif ()

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(Eigen3 REQUIRED QUIET)
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 3 REQUIRED QUIET)
endif ()
message(STATUS "OPENCV: " ${OpenCV_VERSION})
message(STATUS "Eigen3: " ${Eigen3_VERSION})

# GCC older than 9.1 needs an explicit lib for std::filesystem (ROS Melodic ships GCC 7.5)
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.1)
    list(APPEND thirdparty_libraries stdc++fs)
endif ()

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
