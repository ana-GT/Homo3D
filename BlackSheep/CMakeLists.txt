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


set( SRC_FILES   WeightedSearch.cpp
				 BlackSheep.cpp )

set ( HDR_FILES 
				 BlackSheep.h )

add_executable ( test test.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( test ${PCL_LIBRARIES} )	

add_executable ( test2  test2.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( test2 ${PCL_LIBRARIES} )	

add_executable ( test3  test3.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( test3 ${PCL_LIBRARIES} )	

add_executable ( testWeighted  testWeighted.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( testWeighted ${PCL_LIBRARIES} )	

add_executable ( testWeighted2  testWeighted2.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( testWeighted2 ${PCL_LIBRARIES} )	

add_executable ( testWeighted3  testWeighted3.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( testWeighted3 ${PCL_LIBRARIES} )	

add_executable ( testWeighted4  testWeighted4.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( testWeighted4 ${PCL_LIBRARIES} )	

add_executable ( testWeighted5  testWeighted5.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( testWeighted5 ${PCL_LIBRARIES} )	

add_executable ( DrawConnectedSet  DrawConnectedSet.cpp ${SRC_FILES} ${HDR_FILES} )
target_link_libraries( DrawConnectedSet ${PCL_LIBRARIES} )	
