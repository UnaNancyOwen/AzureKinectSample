cmake_minimum_required( VERSION 3.6 )

# Language
enable_language( CXX )

# Compiler Settings
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Project
project( transformation LANGUAGES CXX )
add_executable( transformation util.h kinect.hpp kinect.cpp main.cpp )

# (Option) Start-Up Project for Visual Studio
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "transformation" )

# Find Package
find_package( OpenCV REQUIRED )
find_package( k4a REQUIRED )

# Set Package to Project
if( k4a_FOUND AND OpenCV_FOUND )
  target_link_libraries( transformation k4a::k4a )
  target_link_libraries( transformation ${OpenCV_LIBS} )
endif()
