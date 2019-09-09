cmake_minimum_required( VERSION 3.6 )

# Language
enable_language( CXX )

# Compiler Settings
set( CMAKE_CXX_STANDARD 17 ) # require C++17 (or later) for filesystem
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Compiler Option
set( FILESYSTEM )
if( "${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" )
  set( FILESYSTEM "stdc++fs" )
elseif( "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" )
  set( FILESYSTEM "c++fs" )
endif()

# Project
project( playback LANGUAGES CXX )
add_executable( playback util.h kinect.hpp kinect.cpp main.cpp )

# (Option) Start-Up Project for Visual Studio
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "playback" )

# Find Package
find_package( OpenCV REQUIRED )
find_package( k4a REQUIRED )
find_package( k4arecord REQUIRED )

# Set Package to Project
if( k4a_FOUND AND k4arecord_FOUND AND OpenCV_FOUND )
  target_link_libraries( playback k4a::k4a )
  target_link_libraries( playback k4a::k4arecord )
  target_link_libraries( playback ${OpenCV_LIBS} )
  target_link_libraries( playback ${FILESYSTEM} )
endif()
