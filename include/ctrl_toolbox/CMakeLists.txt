# RML: Robotic Matematical Library

# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(ctrl_toolbox VERSION 1.0 LANGUAGES CXX)

# Must use GNUI stallDirs to install libraries into correct
# locations on all platforms.
include(GNUInstallDirs)

# Default to C++17
#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
#endif()

#if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()


option(BUILD_TESTS "Compile tests" OFF)

add_library(ctrl_toolbox SHARED
    src/pid/DigitalPID.cc
    src/virtual_frame/VirtualFrame.cc
    src/HelperFunctions.cc
    src/kalman_filter/ModelKalmanFilter.cc
    src/kalman_filter/ExtendedKalmanFilter.cc
    src/kalman_filter/MeasurementKalmanFilter.cc
)

target_link_libraries(ctrl_toolbox rml Geographic)

set_target_properties(ctrl_toolbox PROPERTIES LINKER_LANGUAGE CXX)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(ctrl_toolbox PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ctrl_toolbox>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ctrl_toolbox_internal>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(ctrl_toolbox
    PUBLIC
        cxx_auto_type
        cxx_range_for
        cxx_variadic_templates
)

# Depend on a library that we defined in the top-level file
#target_link_libraries(lib
#    boost
#    MyOtherLibrary)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS ctrl_toolbox EXPORT ctrl_toolboxConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ctrl_toolbox DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ctrl_toolbox_internal DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT ctrl_toolboxConfig DESTINATION share/ctrl_toolbox/cmake)

# This makes the project importable from the build directory
export(TARGETS ctrl_toolbox FILE ctrl_toolboxConfig.cmake)


if(BUILD_TESTS)
    # Every library has unit tests, of course
    add_executable(ctrl_toolbox_test test/ctrl_toolbox_test.cc)
    
    target_link_libraries(ctrl_toolbox_test ctrl_toolbox)
    
    add_test(ctrl_toolbox_test ctrl_toolbox_test)
endif(BUILD_TESTS)
