# Homotopic Paths 2D

cmake_minimum_required( VERSION 2.8 )
project ( TG3D )

find_package( PCL 1.1 REQUIRED )

include_directories( /usr/include/eigen3
					 ${PCL_INCLUDE_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} ) 

message ( STATUS "Creating Makefile for Homotopic Paths 3D" )
set( CMAKE_CXX_FLAGS "-L /usr/local/lib" )


set( SRC_FILES 
				 Grid3D.cpp
				 TopologyGrid3D.cpp
				 Search.cpp
				 kdtree/kdtree.c
				 test.cpp )

set ( HDR_FILES 
				 Grid3D.h
				 TopologyGrid3D.h
				 Search.h
				 test.h )

add_executable ( test ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( test ${PCL_LIBRARIES} )	

