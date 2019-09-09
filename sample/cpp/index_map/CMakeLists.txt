cmake_minimum_required( VERSION 3.6 )

# Language
enable_language( CXX )

# Compiler Settings
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Project
project( index_map LANGUAGES CXX )
add_executable( index_map util.h kinect.hpp kinect.cpp main.cpp )

# (Option) Start-Up Project for Visual Studio
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "index_map" )

# Find Package
find_package( OpenCV REQUIRED )
find_package( k4a REQUIRED )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" )
find_package( k4abt REQUIRED )

# Set Package to Project
if( k4a_FOUND AND k4abt_FOUND AND OpenCV_FOUND )
  target_link_libraries( index_map k4a::k4a )
  target_link_libraries( index_map k4a::k4abt )
  target_link_libraries( index_map ${OpenCV_LIBS} )
endif()
