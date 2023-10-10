

# Find Python, make sure we use the same version for interpreter and libraries
# First, allow add the user-specified version of python to our build
set(Python_ADDITIONAL_VERSIONS ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR})

find_package(PythonInterp)
message("Python version from interpreter is ${PYTHON_VERSION_STRING}")
find_package(PythonLibs)

# Helper function to search for a module
function(find_python_module module)
	string(TOUPPER ${module} module_upper)
	if(NOT PY_${module_upper})
		if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
			set(${module}_FIND_REQUIRED TRUE)
		endif()
		# A module's location is usually a directory, but for binary modules
		# it's a .so file.
		execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
			"import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
			RESULT_VARIABLE _${module}_status
			OUTPUT_VARIABLE _${module}_location
			ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
		if(NOT _${module}_status)
			set(PY_${module_upper} ${_${module}_location} CACHE STRING
				"Location of Python module ${module}")
		endif(NOT _${module}_status)
	endif(NOT PY_${module_upper})
endfunction(find_python_module)

# Make sure we have the right modules
if (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND)
	message("Python libs and executable found, looking for numpy and boost::python")

	if (PYTHON_DEB_INSTALL_TARGET)
		set(BOOST_PYTHON_COMPONENT "python-py${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}")
	else (PYTHON_DEB_INSTALL_TARGET)
		if (PYTHON_VERSION_MAJOR GREATER 2)
			set(BOOST_PYTHON_COMPONENT "python${PYTHON_VERSION_MAJOR}")
		else (PYTHON_VERSION_MAJOR GREATER 2)
			set(BOOST_PYTHON_COMPONENT "python")
		endif(PYTHON_VERSION_MAJOR GREATER 2)
	endif (PYTHON_DEB_INSTALL_TARGET)

	find_package(Boost COMPONENTS ${BOOST_PYTHON_COMPONENT})
	find_python_module(numpy)
	find_package_handle_standard_args(numpy DEFAULT_MSG PY_NUMPY)
	if (Boost_FOUND AND NUMPY_FOUND)
		message("numpy and boost::python found, generating python bindings")
		include_directories(${PYTHON_INCLUDE_DIRS} ${PY_NUMPY}/core/include)
		if (SHARED_LIBS)
			python_add_module(pynabo SHARED nabo.cpp)
			target_link_libraries(pynabo ${LIB_NAME} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
		else (SHARED_LIBS)
			set(PYTHON_SRC "nabo.cpp")
			foreach(file ${NABO_SRC})
				set(PYTHON_SRC ${PYTHON_SRC} "../${file}")
			endforeach(file)
			python_add_module(pynabo ${PYTHON_SRC})
			target_link_libraries(pynabo ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
		endif (SHARED_LIBS)
		# fix for old python_add_module
		set_target_properties(pynabo PROPERTIES PREFIX "")
		if (PYTHON_CUSTOM_TARGET)
			install(TARGETS pynabo LIBRARY DESTINATION ${PYTHON_CUSTOM_TARGET})
		else (PYTHON_CUSTOM_TARGET)
			if (PYTHON_DEB_INSTALL_TARGET)
				set(PYTHON_COMMAND "import sys; print('/usr/lib/python'+str(sys.version_info[0])+'.'+str(sys.version_info[1])+'/dist-packages')")
			else (PYTHON_DEB_INSTALL_TARGET)
				set(PYTHON_COMMAND "from distutils.sysconfig import get_python_lib; print(get_python_lib(prefix='${CMAKE_INSTALL_PREFIX}'))")
			endif (PYTHON_DEB_INSTALL_TARGET)
			execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c" "${PYTHON_COMMAND}" OUTPUT_VARIABLE PYTHON_SITE_MODULES OUTPUT_STRIP_TRAILING_WHITESPACE)
			install(TARGETS pynabo LIBRARY DESTINATION ${PYTHON_SITE_MODULES})
		endif (PYTHON_CUSTOM_TARGET)
	elseif (Boost_FOUND AND NUMPY_FOUND)
		message("You need numpy and boost::python to generate python bindings")
	endif (Boost_FOUND AND NUMPY_FOUND)
else (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND)
	message("Python libs or executable not found, skipping Python bindings")
endif (PYTHONLIBS_FOUND AND PYTHONINTERP_FOUND)
