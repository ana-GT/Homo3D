/**
 * @function Grid3D.cpp
 * @brief Functions for Grid2D.cpp
 */

//-- PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Grid3D.h"

/**
 * @function Grid3D
 * @brief Constructor	
 */
Grid3D::Grid3D( int _sizeX, int _sizeY, int _sizeZ ) {

	mSizeX = _sizeX;
	mSizeY = _sizeY;
	mSizeZ = _sizeZ;
    mSize = mSizeX*mSizeY*mSizeZ;
	mState = new bool[mSize]; 
	mStride1 = mSizeY*mSizeZ;
	mStride2 = mSizeZ;

    /// Set all cells to free: false
	for( unsigned int i = 0; i < mSizeX; i++ ) {	
		for( unsigned int j = 0; j < mSizeY; j++ ) {
			for( unsigned int k = 0; k < mSizeZ; ++k ) {
				Pos p; 
            	p.x = i; p.y = j; p.z = k;
				SetState( p, false );
			}
		}
	}	

}


/**
 * @function Grid3D
 * @brief Destructor	
 */
Grid3D::~Grid3D() {
	delete [] mState;
}

/**
 * @function GetState
 */
bool Grid3D::GetState( Pos _p ) const {
	return mState[ ref(_p) ];
}

/**
 * @function GetState
 */
bool Grid3D::GetState( int _x, int _y, int _z ) const {
	return mState[ ref(_x, _y, _z) ];
}

/**
 * @function CheckCollision
 */
bool Grid3D::CheckCollision( Pos _p ) const {
	if( GetState( _p ) == false ) { // free
		return false;
	}
	else { // occupied
		return true;
	}
}

/**
 * @function SetCell
 */
void Grid3D::SetState( Pos _p, bool _state ) {
	mState[ref(_p)] = _state;
}

/**
 * @function SetState
 */
void Grid3D::SetState( int _x, int _y, int _z, bool _state ) {
	mState[ref(_x, _y, _z)] = _state;
}


//////////// UTILITY ////////////////

/**
 * @function CreateBorderingObstacles
 */
void Grid3D::CreateExternalBoundary() {

	//-- Add the faces
	for( size_t j = 0; j < mSizeY; ++j ) {
		for( size_t k = 0; k < mSizeZ; ++k ) {
			SetState( 0, j, k, true );			// Face 2
			SetState( mSizeX - 1, j, k, true );	// Face 4
		}	
	}

	for( size_t i = 0; i < mSizeX; ++i ) {
		for( size_t k = 0; k < mSizeZ; ++k ) {
			SetState( i, 0, k, true );			// Face 1
			SetState( i, mSizeY - 1, k, true );	// Face 3
		}	
	}

	for( size_t i = 0; i < mSizeX; ++i ) {
		for( size_t j = 0; j < mSizeY; ++j ) {
			SetState( i, j, 0, true );			// Face 6
			SetState( i, j, mSizeZ - 1, true );	// Face 5
		}	
	}

}

/**
 * @function CreateBox
 */
void Grid3D::CreateBox( int _x, int _y, int _z, int _dimX, int _dimY, int _dimZ ) {

	for( int i = _x; i < _x + _dimX; ++i ) {
		for( int j = _y; j < _y + _dimY; ++j ) {
			for( int k = _z; k < _z + _dimZ; ++k ) {
				SetState( i, j, k, true );
			}
		}
	}	

}

/**
 * @function Visualization
 */
void Grid3D::Visualization() {
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects( new pcl::PointCloud<pcl::PointXYZRGB> );

	objects->height = 1;
	objects->is_dense = false;
	objects->points.resize( 0 );

	uint8_t r = 255; uint8_t g =  0; uint8_t b = 0;			
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

    int count = 0;


	pcl::PointXYZRGB q;
		
	for( unsigned int i = 0; i < mSizeX; i++ ) {	
		for( unsigned int j = 0; j < mSizeY; j++ ) {
			for( unsigned int k = 0; k < mSizeZ; ++k ) {
				if( GetState( i, j, k ) == true ) { // Obstacle
					q.x = (double) i;
					q.y = (double) j;
					q.z = (double) k;				
					q.rgb = *reinterpret_cast<float*>(&rgb);
					objects->points.push_back(q); count++;        
					++count;
				}
			}
		}
	}

	printf( "--> Number of grids with obstacles: %d \n", count );
	pcl::visualization::CloudViewer viewer( "Grid3D Visualization" );
	viewer.showCloud( objects );
	while( !viewer.wasStopped() ) {		
	}	

}
