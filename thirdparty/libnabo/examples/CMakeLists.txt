include_directories(..)

add_executable(trivial trivial.cpp)
target_link_libraries(trivial ${LIB_NAME} ${EXTRA_LIBS})

add_executable(usage usage.cpp)
target_link_libraries(usage ${LIB_NAME} ${EXTRA_LIBS})
