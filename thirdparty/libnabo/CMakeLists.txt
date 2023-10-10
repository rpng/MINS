cmake_minimum_required(VERSION 2.6)

if (NOT CMAKE_VERSION VERSION_LESS "3.1")
	cmake_policy(SET CMP0054 NEW)
endif ()

set(LIB_NAME nabo)
project("lib${LIB_NAME}")

# Extract version from header
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
execute_process(
	COMMAND grep "NABO_VERSION " nabo/nabo.h
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	RESULT_VARIABLE GREP_VERSION_RESULT
	OUTPUT_VARIABLE PROJECT_VERSION
	OUTPUT_STRIP_TRAILING_WHITESPACE
)
if (NOT GREP_VERSION_RESULT EQUAL 0)
	message(SEND_ERROR "Cannot grep version number: ${GREP_VERSION_RESULT}")
endif (NOT GREP_VERSION_RESULT EQUAL 0)
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" PROJECT_VERSION "${PROJECT_VERSION}" )

if (NOT CMAKE_BUILD_TYPE)
	message("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
	set(CMAKE_BUILD_TYPE Release CACHE STRING
	  "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	  FORCE)
else (NOT CMAKE_BUILD_TYPE)
	if (CMAKE_BUILD_TYPE STREQUAL "Debug")
		message("\n=================================================================================")
		message("\n-- Build type: Debug. Performance will be terrible!")
		message("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
		message("\n=================================================================================")
	endif (CMAKE_BUILD_TYPE STREQUAL "Debug")
endif (NOT CMAKE_BUILD_TYPE)

if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
	add_definitions(-O3)
endif(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")

# Documentation
set(DOXYFILE_LATEX false)
include(UseDoxygen)

# Switch on warnings.
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")
else ()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif ()

# Check the compiler version as we need C++11 support.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	# using Clang
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.3")
		message(FATAL_ERROR "Your clang compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while version 3.3 or later is required")
	endif ()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
	# using AppleClang
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.1")
		message(FATAL_ERROR "Your XCode environment has version ${CMAKE_CXX_COMPILER_VERSION}, while version 5.1 or later is required")
	endif ()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	# using GCC
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.8.2")
		message(FATAL_ERROR "Your g++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while version 4.8.2 or later is required")
	endif ()
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	# using MSVC
	if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "19.0.23506")
		message(FATAL_ERROR "Your Microsoft Visual C++ compiler has version ${CMAKE_CXX_COMPILER_VERSION}, while version MSVC 2015 Update 1+ is required")
	endif ()
endif ()

# enable C++11 support.
if (CMAKE_VERSION VERSION_LESS "3.1")
	if (MSVC)
		message(FATAL_ERROR "CMake version 3.1 or later is required to compile ${PROJECT_NAME} with Microsoft Visual C++")
	endif ()
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set (CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
	else ()
		set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
	endif ()
else ()
	set (CMAKE_CXX_STANDARD 11)
endif ()

# openmp
set(USE_OPEN_MP TRUE CACHE BOOL "Set to FALSE to not use OpenMP")
if (USE_OPEN_MP)
	find_package(OpenMP)
	if (OPENMP_FOUND)
		add_definitions(-fopenmp -DHAVE_OPENMP)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		if (CMAKE_COMPILER_IS_GNUCC)
			set(EXTRA_LIBS ${EXTRA_LIBS} gomp)
		endif(CMAKE_COMPILER_IS_GNUCC)
	endif(OPENMP_FOUND)
endif (USE_OPEN_MP)

# eigen 2 or 3
find_path(EIGEN_INCLUDE_DIR Eigen/Core
	/usr/local/include/eigen3
	/usr/local/include/eigen2
	/usr/local/include/eigen
	/usr/include/eigen3
	/usr/include/eigen2
	/usr/include/eigen
	/opt/local/include/eigen3
)

# optionally, opencl
# OpenCL disabled as its code is not up-to-date with API
set(USE_OPEN_CL FALSE CACHE BOOL "Set to TRUE to look for OpenCL")
if (USE_OPEN_CL)
	find_path(OPENCL_INCLUDE_DIR CL/cl.h
		/usr/local/include
		/usr/include
	)
	if (WIN32)
		find_library(OPENCL_LIBRARIES opencl64)
		if (!OPENCL_LIBRARIES)
			find_library(OPENCL_LIBRARIES opencl32)
		endif (!OPENCL_LIBRARIES)
	else (WIN32)
		find_library(OPENCL_LIBRARIES OpenCL ENV LD_LIBRARY_PATH)
	endif (WIN32)
	if (OPENCL_INCLUDE_DIR AND OPENCL_LIBRARIES)
		add_definitions(-DHAVE_OPENCL)
		set(EXTRA_LIBS ${OPENCL_LIBRARIES} ${EXTRA_LIBS})
		include_directories(${OPENCL_INCLUDE_DIR})
		add_definitions(-DOPENCL_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}/nabo/opencl/\")
		message("OpenCL enabled and found, enabling CL support")
	else (OPENCL_INCLUDE_DIR AND OPENCL_LIBRARIES)
		message("OpenCL enabled but not found, disabling CL support")
	endif (OPENCL_INCLUDE_DIR AND OPENCL_LIBRARIES)
else (USE_OPEN_CL)
	message("OpenCL disabled, not looking for it")
endif (USE_OPEN_CL)

# include all libs so far
include_directories(${CMAKE_SOURCE_DIR} ${EIGEN_INCLUDE_DIR})

# main nabo lib
set(NABO_SRC
	nabo/nabo.cpp
	nabo/brute_force_cpu.cpp
	nabo/kdtree_cpu.cpp
	nabo/kdtree_opencl.cpp
)
set(SHARED_LIBS FALSE CACHE BOOL "Set to TRUE to build shared library")
if (SHARED_LIBS)
	add_library(${LIB_NAME} SHARED ${NABO_SRC})
	install(TARGETS ${LIB_NAME} LIBRARY DESTINATION lib)
else (SHARED_LIBS)
	add_library(${LIB_NAME} ${NABO_SRC})
	add_definitions(-fPIC)
	install(TARGETS ${LIB_NAME} ARCHIVE DESTINATION lib)
endif (SHARED_LIBS)
set_target_properties(${LIB_NAME} PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

# create doc before installing
set(DOC_INSTALL_TARGET "share/doc/${PROJECT_NAME}/api" CACHE STRING "Target where to install doxygen documentation")
install(FILES nabo/nabo.h DESTINATION include/nabo)
install(FILES nabo/third_party/any.hpp DESTINATION include/nabo/third_party)
install(FILES README.md DESTINATION share/doc/${PROJECT_NAME})
if (DOXYGEN_FOUND)
	install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/html DESTINATION ${DOC_INSTALL_TARGET})
endif (DOXYGEN_FOUND)

enable_testing()

add_subdirectory(examples)
add_subdirectory(tests)
add_subdirectory(python)

# Install catkin package.xml
install(FILES package.xml DESTINATION share/libnabo)

#=============================================
# to allow find_package() on libnabo
#=============================================
# 
# the following case be used in an external project requiring libnabo:
#  ...
#  find_package(libnabo) 
#  include_directories(${libnabo_INCLUDE_DIRS}) 
#  target_link_libraries(executableName ${libnabo_LIBRARIES})
#  ...

# NOTE: the following will support find_package for 1) local build (make) and 2) for installed files (make install)

# 1- local build #

# Register the local build in case one doesn't use "make install"
export(PACKAGE libnabo)

# Create variable with the library location
set(libnabo_library $<TARGET_FILE:${LIB_NAME}>)

# Create variable for the local build tree
get_property(libnabo_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
# Configure & generate config file for local build tree
configure_file(libnaboConfig.cmake.in
	"${PROJECT_BINARY_DIR}/libnaboConfig.cmake.conf" @ONLY)
file(GENERATE
	OUTPUT "${PROJECT_BINARY_DIR}/libnaboConfig.cmake"
	INPUT "${PROJECT_BINARY_DIR}/libnaboConfig.cmake.conf")

# 2- installation build #

# Change the library location for an install location
set(libnabo_library ${CMAKE_INSTALL_PREFIX}/lib/$<TARGET_FILE_NAME:${LIB_NAME}>)

# Change the include location for the case of an install location
set(libnabo_include_dirs ${CMAKE_INSTALL_PREFIX}/include)

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(libnaboConfig.cmake.in
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libnaboConfig.cmake.conf" @ONLY)
file(GENERATE
	OUTPUT "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libnaboConfig.cmake"
	INPUT "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libnaboConfig.cmake.conf")

# The same versioning file can be used for both cases
configure_file(libnaboConfigVersion.cmake.in
	"${PROJECT_BINARY_DIR}/libnaboConfigVersion.cmake" @ONLY)

install(FILES
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libnaboConfig.cmake"
	"${PROJECT_BINARY_DIR}/libnaboConfigVersion.cmake"
	DESTINATION share/libnabo/cmake COMPONENT dev)


#=============================================
# Add uninstall target
#=============================================
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
	IMMEDIATE @ONLY)

add_custom_target(uninstall
	COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
