# Homotopic Paths 3D

cmake_minimum_required( VERSION 2.8 )
project ( HP3D )

find_package( PCL 1.1 REQUIRED )

include_directories( distance_field/include
					 /usr/include/eigen3
					 ${PCL_INCLUDE_DIRS} )

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} ) 


message ( STATUS "Creating Makefile for Homotopic Paths 3D" )
set( CMAKE_CXX_FLAGS "-L /usr/local/lib" )


set( SRC_FILES
			    distance_field/src/pf_distance_field.cpp
			    distance_field/src/collision_map.cpp
                kdtree/kdtree.c
				HomoSearch.cpp
				test.cpp )

set( HDR_FILES 
				distance_field/include/distance_field/voxel_grid.h
				distance_field/include/distance_field/distance_field.h
				distance_field/include/distance_field/pf_distance_field.h
				distance_field/include/distance_field/collision_map.h
				HomoSearch.h )

add_executable( test ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( test ${PCL_LIBRARIES} )				
