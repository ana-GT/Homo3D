# Homotopic Paths 2D

cmake_minimum_required( VERSION 2.8 )
project ( TG2D )

message ( STATUS "Creating Makefile for Homotopic Paths 2D" )
set( CMAKE_CXX_FLAGS "-L /usr/local/lib" )

include_directories( /usr/include/eigen3 )

set( SRC_FILES 
				 Grid2D.cpp
				 TopologyGrid2D.cpp
				 Search.cpp
				 kdtree/kdtree.c
				 SDL_Functions.cpp
				 testSDL.cpp )

set ( HDR_FILES 
				 Grid2D.h
				 TopologyGrid2D.h
				 Search.h
				 testSDL.h )

set( SDL_LIBS SDL
			  SDL_image )


add_executable ( test ${SRC_FILES} ${HDR_FILES} )
target_link_libraries ( test ${SDL_LIBS}  )
