include (CheckSymbolExists) 

include_directories(..)

add_executable(knnvalidate knnvalidate.cpp)
target_link_libraries(knnvalidate ${LIB_NAME} ${EXTRA_LIBS})
# disable as experimental is broken target_link_libraries(knnvalidate naboexperimental nabo  ${EXTRA_LIBS})

add_test(validation-2D-exhaustive ${EXECUTABLE_OUTPUT_PATH}/knnvalidate ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.2d.txt 10 2 -1)
add_test(validation-2D-random ${EXECUTABLE_OUTPUT_PATH}/knnvalidate ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.2d.txt 10 2 10000)
add_test(validation-3D-exhaustive ${EXECUTABLE_OUTPUT_PATH}/knnvalidate ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.txt 10 3 -1)
add_test(validation-3D-random ${EXECUTABLE_OUTPUT_PATH}/knnvalidate ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.txt 10 3 10000)
add_test(validation-3D-large-random ${EXECUTABLE_OUTPUT_PATH}/knnvalidate ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 10 3 1000)
add_test(validation-3D-large-random-radius ${EXECUTABLE_OUTPUT_PATH}/knnvalidate ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 10 3 1000 0.5)

find_path(ANN_INCLUDE_DIR ANN.h
	/usr/local/include/ANN
	/usr/include/ANN
)
find_library(ANN_LIBRARY
	NAMES ANN ann
	PATHS /usr/local/lib
	/usr/lib
)

if (ANN_INCLUDE_DIR AND ANN_LIBRARY)
	include_directories(${ANN_INCLUDE_DIR})
	set(EXTRA_LIBS ${ANN_LIBRARY} ${EXTRA_LIBS})
	add_definitions(-DHAVE_ANN)
	message("ANN found, enabling it in benchmarks")
else (ANN_INCLUDE_DIR AND ANN_LIBRARY)
	message("ANN not found, disabling it in benchmarks")
endif (ANN_INCLUDE_DIR AND ANN_LIBRARY)

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
	/usr/local/include/
	/usr/include
)
find_library(FLANN_LIBRARY
	flann_cpp
	PATHS /usr/local/lib
	/usr/lib
)

if (FLANN_INCLUDE_DIR AND FLANN_LIBRARY)
	include_directories(${FLANN_INCLUDE_DIR})
	set(EXTRA_LIBS ${FLANN_LIBRARY} ${EXTRA_LIBS})
	add_definitions(-DHAVE_FLANN)
	message("FLANN found, enabling it in benchmarks")
else (FLANN_INCLUDE_DIR AND FLANN_LIBRARY)
	message("FLANN not found, disabling it in benchmarks")
endif (FLANN_INCLUDE_DIR AND FLANN_LIBRARY)

include_directories(..)

add_executable(knnbench knnbench.cpp)
target_link_libraries(knnbench ${LIB_NAME} ${EXTRA_LIBS})
# disable as experimental is broken target_link_libraries(knnbench naboexperimental nabo ${EXTRA_LIBS})

add_test(bench-3D-large-exhaustive-10000-K1 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 1 -10000 3 5)
add_test(bench-3D-large-exhaustive-1000-K1 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 1 -1000 3 5)
add_test(bench-3D-large-exhaustive-100-K1 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 1 -100 3 5)
add_test(bench-3D-large-random-K1 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 1 40000 3 5)
add_test(bench-3D-large-exhaustive-10000-K10 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 10 -10000 3 5)
add_test(bench-3D-large-exhaustive-1000-K10 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 10 -1000 3 5)
add_test(bench-3D-large-exhaustive-100-K10 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 10 -100 3 5)
add_test(bench-3D-large-random-K20 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 10 40000 3 5)
add_test(bench-3D-large-exhaustive-10000-K30 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 30 -10000 3 5)
add_test(bench-3D-large-exhaustive-1000-K30 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 30 -1000 3 5)
add_test(bench-3D-large-exhaustive-100-K30 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 30 -100 3 5)
add_test(bench-3D-large-random-K30 ${EXECUTABLE_OUTPUT_PATH}/knnbench ${CMAKE_CURRENT_SOURCE_DIR}/data/scan.3d.large.txt 30 40000 3 5)

add_executable(knnepsilon knnepsilon.cpp)
target_link_libraries(knnepsilon ${LIB_NAME} ${EXTRA_LIBS})

add_executable(knnbucketsize knnbucketsize.cpp)
target_link_libraries(knnbucketsize ${LIB_NAME} ${EXTRA_LIBS})

# Ensure that users cannot instantiate a tree with wrong matrix types.
macro(try_compile_cloud_types MAT_TYPE TREE_TYPE EXPECT)
	try_compile(COMPILE_SUCCEEDED
	              "${CMAKE_CURRENT_BINARY_DIR}"
	              "${CMAKE_CURRENT_SOURCE_DIR}/invalid_matrix_types.cpp"
	              COMPILE_DEFINITIONS "-std=c++11 -D${MAT_TYPE} -D${TREE_TYPE} -I${CMAKE_CURRENT_SOURCE_DIR}/../ -I${CMAKE_CURRENT_SOURCE_DIR}/../third_party/ -I${EIGEN_INCLUDE_DIR} -l${LIB_NAME}"
	              CMAKE_FLAGS "-DCMAKE_CXX_LINK_EXECUTABLE='echo not linking now...'"
	              OUTPUT_VARIABLE OUTPUT)

	if (${EXPECT} STREQUAL "TRUE") 
		if (NOT COMPILE_SUCCEEDED)
			message( FATAL_ERROR "Compiling a test executable failed. This is an error. Message ${OUTPUT}." )
		endif ()
	else ()
		if (COMPILE_SUCCEEDED)
			message( FATAL_ERROR "Tried to instantiate nabo-search trees with non-allowed types and succeeded. This is an error. Message ${OUTPUT}" )
		endif ()
	endif ()
endmacro(try_compile_cloud_types)

set(RUN_COMPILATION_TESTS false CACHE BOOL "Set to true to test the compilation of libnabo with different matrix types. Will only work with GCC and Clang.")

if (RUN_COMPILATION_TESTS AND (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
	try_compile_cloud_types(NABO_EIGEN_DYNAMIC_TYPE NABO_TYPE_CREATE TRUE)
	try_compile_cloud_types(NABO_EIGEN_DYNAMIC_TYPE NABO_TYPE_BRUTE_FORCE TRUE)
	try_compile_cloud_types(NABO_EIGEN_DYNAMIC_TYPE NABO_TYPE_LINEAR_HEAP TRUE)
	try_compile_cloud_types(NABO_EIGEN_DYNAMIC_TYPE NABO_TYPE_TREE_HEAP TRUE)

	try_compile_cloud_types(NABO_EIGEN_SEMI_DYNAMIC_TYPE NABO_TYPE_CREATE FALSE)
	try_compile_cloud_types(NABO_EIGEN_SEMI_DYNAMIC_TYPE NABO_TYPE_BRUTE_FORCE FALSE)
	try_compile_cloud_types(NABO_EIGEN_SEMI_DYNAMIC_TYPE NABO_TYPE_LINEAR_HEAP FALSE)
	try_compile_cloud_types(NABO_EIGEN_SEMI_DYNAMIC_TYPE NABO_TYPE_TREE_HEAP FALSE)
endif ()
