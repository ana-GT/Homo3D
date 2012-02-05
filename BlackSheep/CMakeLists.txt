# Black Sheep

cmake_minimum_required( VERSION 2.8 )
project ( BlackSheep )

find_package( PCL 1.1 REQUIRED )

include_directories( /usr/include/eigen3
					 ${PCL_INCLUDE_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} ) 

message ( STATUS "Creating Makefile for Black Sheep" )
set( CMAKE_CXX_FLAGS "-L /usr/local/lib" )


set( SRC_FILES 
				 BlackSheep.cpp
				 test.cpp )

set ( HDR_FILES 
				 BlackSheep.h )

add_executable ( test ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( test ${PCL_LIBRARIES} )	

