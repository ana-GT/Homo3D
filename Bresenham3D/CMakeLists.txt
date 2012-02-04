cmake_minimum_required( VERSION 2.8 )
project ( BresenhamTest )

include_directories ( /usr/include/eigen3 )

message( STATUS "Creating Makefile for Bresenham Test" )
set ( CMAKE_CXX_FLAGS "-L /usr/local/lib" )

set ( SRC_FILES 
				Bresenham3D.cpp
				testBresenham.cpp )

set ( HDR_FILES
				Bresenham3D.h )

add_executable ( test ${SRC_FILES} ${HDR_FILES} )
#target_link_libraries( test ${} )
