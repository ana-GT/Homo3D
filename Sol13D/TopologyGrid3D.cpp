/**
 * @file TopologyGrid3D.cpp
 */


//-- PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "TopologyGrid3D.h"
#include <stdio.h>
#include <cstdlib>
#include <limits.h>
#include <float.h>


int TopologyGrid3D::NX[26] = {-1,-1,1, 1,-1,-1, 1, 1,-1,-1,1, 1, 0,-1,0,1, 0,-1, 0, 1,-1,0,1, 0,0, 0};
int TopologyGrid3D::NY[26] = {-1, 1,1,-1,-1, 1, 1,-1,-1, 1,1,-1,-1, 0,1,0,-1, 0, 1, 0, 0,1,0,-1,0, 0};
int TopologyGrid3D::NZ[26] = { 1, 1,1, 1,-1,-1,-1,-1, 0, 0,0, 0, 1, 1,1,1,-1,-1,-1,-1, 0,0,0, 0,1,-1};

const double TopologyGrid3D::TG3D_INF = DBL_MAX;
const double TopologyGrid3D::TG3D_SQRT2 = 1.414;
const double TopologyGrid3D::TG3D_SQRT3 = 1.732;	


/**
 * @function Constructor
 */

TopologyGrid3D::TopologyGrid3D( Grid3D *_g ) {

	mG = _g;
	mSizeX = _g->GetSizeX();	
	mSizeY = _g->GetSizeY();
	mSizeZ = _g->GetSizeZ();
	mNumCells = mSizeX*mSizeY*mSizeZ;	
	mStride1 = mSizeY*mSizeZ;
	mStride2 = mSizeZ;

    Cell cell; Pos p; int index;

	//-- Create basic nodes
	index = 0;
	for( int i = 0; i < _g->GetSizeX(); ++i ) {
		for( int j = 0; j <	_g->GetSizeY(); ++j ) {
			for( int k = 0; k <	_g->GetSizeZ(); ++k ) {

				p.x = i; p.y = j; p.z = k;
				cell.pos = p;
				cell.distance = TG3D_INF; // No yet check obstacles

				cell.index = index;
				cell.neighbors = GetNeighbors( p );
				mCells.push_back( cell );
			
				index++;	
			}		
		}		
	}
	
}

/**
 * @function Destructor
 */

TopologyGrid3D::~TopologyGrid3D( ) {

}

/**
 * @function CalculateDT
 */

void TopologyGrid3D::CalculateDT() {

	std::vector<int> queue;
	
	//-- 1. Initialize queue with obstacle grids and set distances to zero for them
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {
			for( int k = 0; k < mSizeZ; ++k ) {
				Pos p; p.x = i; p.y = j; p.z = k;
				int ind = ref( i, j, k );

				if( mG->GetState( p ) == true ) {
					queue.push_back( ind );
					mCells[ ref( p.x, p.y, p.z )].distance = 0;
				}
			}
		}
	}

	//-- 2. Loop until no more new elements in Queue
	std::vector<int> newQueue(0);

	while( queue.size() > 0 ) {

		for( int i = 0; i < queue.size(); ++i ) {
			std::vector<int> n = mCells[ queue[i] ].neighbors;
			double dist = mCells[ queue[i] ].distance;

			for( int j = 0; j < n.size(); ++j ) {
				double new_dist = dist + EdgeCost( queue[i], n[j] );
				if( new_dist < mCells[ n[j] ].distance ) {
					mCells[ n[j] ].distance = new_dist;
					newQueue.push_back( n[j] );
				}
			}
		}

		queue.clear();
		queue = newQueue;
		newQueue.clear();

	}

}

/**
 * @function EdgeCost
 */

double TopologyGrid3D::EdgeCost( int _n1, int _n2 ) {

	Pos p1 = mCells[_n1].pos;
	Pos p2 = mCells[_n2].pos;

    int d = abs( p1.x - p2.x ) + abs( p1.y - p2.y ) + abs( p1.z - p2.z );

	if( d == 3 ) {
		return TG3D_SQRT3; // sqrt(3)
	}

	else if( d == 2 ) {
		return TG3D_SQRT2; // sqrt(2)
	}
	else if( d == 1 ) {
		return 1.0; // 1
	}
	else {
		printf( "--> WTH. There is an error here! \n" );
	}
}

/**
 * @function GetDTRidge
 */

std::vector<Cell> TopologyGrid3D::GetDTRidge() {

	std::vector<Cell> ridge;
	
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {
			for( int k = 0; k < mSizeZ; ++k ) {

				int x = ref( i, j, k );
				if( mG->GetState( i, j, k ) != true && IsLocalMaxima( x ) == true ) {		
					ridge.push_back( mCells[x] );
				}		
			}		
		}
	}	

	return ridge;
}

/**
 * @function GetFreeCells
 */

std::vector<Cell> TopologyGrid3D::GetFreeCells() {

	std::vector<Cell> free;

	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {
			for( int k = 0; k < mSizeZ; ++k ) {
				int x = ref( i, j, k );
				if( mG->GetState( i, j, k ) != true) {		
					free.push_back( mCells[x] );
				}
			}			
		}
	}	

	return free;
	
}


/**
 * @function IsLocalMaximum
 */

bool TopologyGrid3D::IsLocalMaxima( int _index ) {

	Cell c = mCells[ _index ];
	double dist = c.distance;		
	std::vector<int> nx = c.neighbors;

	for( int i = 0; i < nx.size(); ++i ) {

		Cell n = mCells[nx[i]];

		if( EdgeCost( nx[i], _index ) == 1.0 && n.distance >= c.distance + 1.0  ) {
			return false;		
		}
		else if( EdgeCost( nx[i], _index ) == TG3D_SQRT2 && n.distance >= c.distance  + TG3D_SQRT2 ) {		
			return false;
		}
		else if( EdgeCost( nx[i], _index ) == TG3D_SQRT3 && n.distance >= c.distance  + TG3D_SQRT3 ) {		
			return false;
		}
	}

	return true;
}


/**
 * @function GetNeighbors	
 */

std::vector<int> TopologyGrid3D::GetNeighbors( Pos _p ) {

	std::vector<int> neighbors;	
	int x = _p.x;
	int y = _p.y;
	int z = _p.z;
	int nx; int ny; int nz;

	for( int i = 0; i < 26; ++i ) {
		nx = x + NX[i];
		ny = y + NY[i];
		nz = z + NZ[i];
		if( IsValid( nx, ny, nz ) == true ) {
			neighbors.push_back( ref(nx, ny, nz) );
		}
	}		
	return neighbors;		
}

/**
 * @function IsValid
 */
bool TopologyGrid3D::IsValid( int _x, int _y, int _z ) {

	if( _x < 0 || _x >= mSizeX || _y < 0 || _y >= mSizeY || _z < 0 || _z >= mSizeZ ) {
		return false;
	}

	return true;
}

/**
 * @function ref
 */
int TopologyGrid3D::ref( int _x, int _y, int _z ) {
	return _x*mStride1 + _y*mStride2 + _z;
}

/**
 * @function VisualizeRidge
 */
void TopologyGrid3D::VisualizeRidge() {


	std::vector<Cell> ridge = GetDTRidge();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects( new pcl::PointCloud<pcl::PointXYZRGB> );

	objects->height = 1;
	objects->is_dense = false;
	objects->points.resize( 0 );

	uint8_t r = 0; uint8_t g =  0; uint8_t b = 255;			
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

    int count = 0;
	//-- First, find min and max distances
		
	for( size_t i = 0; i < ridge.size() ; ++i ) {

		pcl::PointXYZRGB q;
		q.x = ridge[i].pos.x; 
		q.y = ridge[i].pos.y; 
		q.z = ridge[i].pos.z;					
		q.rgb = *reinterpret_cast<float*>(&rgb);
		objects->points.push_back(q); count++;        
	}


	pcl::visualization::CloudViewer viewer( "Ridge visualization" );
	viewer.showCloud( objects );
	while( !viewer.wasStopped() ) {		
	}	


}
