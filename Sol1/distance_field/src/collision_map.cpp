/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** @author A. Huaman */

#include <distance_field/collision_map.h>
#include <stdio.h>
#include <iostream>
#include <string>


namespace distance_field {

/**
 * @function CollisionMap
 * @brief Constructor
 */
CollisionMap::CollisionMap( double _size_x, 
				  			double _size_y,
				  			double _size_z,
				  			double _resolution,
				  			double _origin_x,
				  			double _origin_y,
 				  			double _origin_z, 
				  			VoxelState _default_object ):
VoxelGrid<VoxelState>( _size_x, 
				 	   _size_y, 
				       _size_z,
			           _resolution, 
				       _origin_x, 
                       _origin_y, 
                       _origin_z, 
                       _default_object )
{
	//-- Reset the cells to FREE state
	VoxelState st = FREE;
	reset(st);
}

/**
 * @function ~CollisionMap
 * @brief Destructor
 */
CollisionMap::~CollisionMap() {

}

/**
 * @function visualize
 */
void CollisionMap::visualize() {
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects( new pcl::PointCloud<pcl::PointXYZRGB> );

	//-- Load 3D environment info of objects into a PointCloud

	objects->width = num_cells_total_;
	objects->height = 1;
	objects->is_dense = false;
	objects->points.resize( 0 );

	int index = 0;

	//-- Define color for objects - Magenta
	uint8_t r = 255; uint8_t g = 0; uint8_t b = 255;
	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	

	//-- Set the occupied cells
	for( size_t i = 0; i < num_cells_[DIM_X] ; ++i ) {
		for( size_t j = 0; j < num_cells_[DIM_Y]; ++j ) {
			for( size_t k = 0; k < num_cells_[DIM_Z]; ++k ) {
				if( getCell( i, j, k ) == OCCUPIED ) {
					pcl::PointXYZRGB p;
					p.x = getLocationFromCell( DIM_X, i );
					p.y = getLocationFromCell( DIM_Y, j );
					p.z = getLocationFromCell( DIM_Z, k );
					p.rgb = *reinterpret_cast<float*>(&rgb);
					objects->points.push_back(p);
				    ++index;
				}

			}
		}
		
	}
	printf("Occupied cells: %d \n", index );
	pcl::visualization::CloudViewer viewer( "Visualization Collision Map" );
	viewer.showCloud( objects );
	while( !viewer.wasStopped() ) {
		
	}
	
}

//-- Utility functions

/**
 * @function addExternalBoundary
 */
bool CollisionMap::addExternalBoundary() {
	
	int Mx = getNumCells( DIM_X);
	int My = getNumCells( DIM_Y);
	int Mz = getNumCells( DIM_Z);

	//-- Set the needed grid cells to OCCUPIED state
	VoxelState st = OCCUPIED;

	//-- Add the faces
	for( size_t j = 0; j < My; ++j ) {
		for( size_t k = 0; k < Mz; ++k ) {
			setCell( 0, j, k, st );			// Face 2
			setCell( Mx - 1, j, k, st );	// Face 4
		}	
	}

	for( size_t i = 0; i < Mx; ++i ) {
		for( size_t k = 0; k < Mz; ++k ) {
			setCell( i, 0, k, st );			// Face 1
			setCell( i, My - 1, k, st );	// Face 3
		}	
	}
// /*
	for( size_t i = 0; i < Mx; ++i ) {
		for( size_t j = 0; j < My; ++j ) {
			setCell( i, j, 0, st );			// Face 6
			setCell( i, j, Mz - 1, st );	// Face 5
		}	
	}
// */

}

/**
 * @function addBox
 */
bool CollisionMap::addBox( double _size_x, double _size_y, double _size_z,
						   double _origin_x,
						   double _origin_y,
						   double _origin_z ) {
	printf("Size cells: %d %d %d \n", getNumCells(DIM_X), getNumCells(DIM_Y), getNumCells(DIM_Z));
	int ox; int oy; int oz;
	int ex; int ey; int ez;
    int nx; int ny; int nz;
	bool status;

	//-- Check that the straight cube is inside the limits of the Collision Map
	if( worldToGrid( _origin_x, _origin_y, _origin_z, ox, oy, oz ) == false ) {
		throw std::string(" (!) Error: Origin of box out of scope \n");
		return false;
	}

    nx = ceil( _size_x / resolution_[DIM_X] );
	ny = ceil( _size_y / resolution_[DIM_Y] );
	nz = ceil( _size_z / resolution_[DIM_Z] );

    ex = ox + nx - 1;
    ey = oy + ny - 1;
    ez = oz + nz - 1;

	if( isCellValid( ex, ey, ez ) == false ) {
		throw std::string(" (!) Error: Extreme of box out of scope");
		return false;
	}

		
		printf("Start: %d %d %d \n", ox, oy, oz);
		printf("End: %d %d %d \n", ex, ey, ez);

	//-- Set the needed grid cells to OCCUPIED state
	VoxelState st = OCCUPIED;

	for( int i = ox; i <= ex; ++i ) {	
		for( int j = oy; j <= ey; ++j ) {
			for( int k = oz; k <= ez; ++k ) {
				setCell( i, j, k, st );
			}
		}
	} 

	return true;
}

} // namespace distance_field



