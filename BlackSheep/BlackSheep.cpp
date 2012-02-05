/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved	
 * Author(s): Ana C. Huaman Quispe <ahuaman3@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */	

#include "BlackSheep.h"
#include <cfloat>
#include <cmath>

const float BS::BS_INF = FLT_MAX;
const float BS::BS_SQRT2 = sqrt(2);
const float BS::BS_SQRT3 = sqrt(3);	
const int BS::NX[26] = {-1,-1,1, 1,-1,-1, 1, 1,-1,-1,1, 1, 0,-1,0,1, 0,-1, 0, 1,-1,0,1, 0,0, 0};
const int BS::NY[26] = {-1, 1,1,-1,-1, 1, 1,-1,-1, 1,1,-1,-1, 0,1,0,-1, 0, 1, 0, 0,1,0,-1,0, 0};
const int BS::NZ[26] = { 1, 1,1, 1,-1,-1,-1,-1, 0, 0,0, 0, 1, 1,1,1,-1,-1,-1,-1, 0,0,0, 0,1,-1};

/**
 * @function BS
 * @brief Constructor
 */
BS::BS( int _sizeX, int _sizeY, int _sizeZ ) {

	mSizeX = _sizeX;
	mSizeY = _sizeY;
	mSizeZ = _sizeZ;

	mStride1 = mSizeY*mSizeZ;
	mStride2 = mSizeZ;

	mNumNodes = mSizeX*mSizeY*mSizeZ;

	mNodes = new Node3D[mNumNodes];

	//-- Create nodes
	int index;
	for( size_t i = 0; i < mSizeX; ++i ) {
		for( size_t j = 0; j < mSizeY; ++j ) {
			for( size_t k = 0; k < mSizeZ; ++k ) {
				Node3D node;
				node.x = i; node.y = j; node.z = k;
				node.obsDist = BS_INF;
				node.state = FREE_STATE;
				index = ref(i,j,k);
				node.index = index;
				// push it
				mNodes[index] = node;

			}
		}
	}
}
		

/**
 * @function ~BS
 * @brief Destructor
 */		
BS::~BS() {

	if( mNodes != NULL ) {
		delete [] mNodes;
	}
}

/**
 * @function SetState
 */
void BS::SetState( int _x, int _y, int _z, int _state ) {

	mNodes[ref(_x, _y, _z)].state = _state;
}

/**
 * @function GetState
 */
int BS::GetState( int _x, int _y, int _z ) const{
	return mNodes[ref(_x, _y, _z)].state;
}

/**
 * @function CheckCollision
 */
bool BS::CheckCollision( int _x, int _y, int _z ) {
	if( GetState(_x, _y, _z) == FREE_STATE ) {
		return false; 
	} else {
		return true;
	}
}


/////////////////// DT Functions ///////////////////////////

/**
 * @function IsValid
 */
bool BS::IsValid( int _x, int _y, int _z ) const {
	if( _x < 0 || _x >= mSizeX || _y < 0 || _y >= mSizeY || _z < 0 || _z >= mSizeZ ) {
		return false;
	}
	return true;
}

/**
 * @function EdgeCost
 */
double BS::EdgeCost( int _index1, int _index2 ) {

	Node3D n1 = mNodes[_index1];
	Node3D n2 = mNodes[_index2];

    int d = abs( n1.x - n2.x ) + abs( n1.y - n2.y ) + abs( n1.z - n2.z );

	if( d == 3 ) {
		return BS_SQRT3; // sqrt(3)
	}

	else if( d == 2 ) {
		return BS_SQRT2; // sqrt(2)
	}
	else if( d == 1 ) {
		return 1.0; // 1
	}
	else {
		printf( "--> WTH. There is an error here! \n" );
		return BS_INF;
	}
}

/**
 * @function GetNeighbors
 */
std::vector<int> BS::GetNeighbors( int _x, int _y, int _z ) const {
	return GetNeighbors( ref( _x, _y, _z ) );
}

/**
 * @function GetNeighbors
 */
std::vector<int> BS::GetNeighbors( int _ref ) const {

	std::vector<int> neighbors;	

	Node3D node = mNodes[_ref];
	int x = node.x;
	int y = node.y;
	int z = node.z;
	int nx; int ny; int nz;

	for( int i = 0; i < 26; ++i ) {
		nx = x + NX[i];
		ny = y + NY[i];
		nz = z + NZ[i];
		if( IsValid( nx, ny, nz ) == true && GetState(nx, ny, nz) == FREE_STATE ) {
			neighbors.push_back( ref(nx, ny, nz) );
		}
	}		
	return neighbors;
}

/**
 * @function
 */
void BS::CalculateDT() {

	printf("--) Calculating DT \n");

	std::vector<int> queue;
	
	//-- 1. Initialize queue with obstacle grids and set distances to zero for them
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {
			for( int k = 0; k < mSizeZ; ++k ) {

				if( GetState( i, j, k ) == OBSTACLE_STATE ) {
					int ind = ref( i, j, k );
					queue.push_back( ind );
					mNodes[ ind ].obsDist = 0;
				}
			}
		}
	}

	//-- 2. Loop until no more new elements in Queue
	std::vector<int> newQueue(0);

	while( queue.size() > 0 ) {

		for( int i = 0; i < queue.size(); ++i ) {
			std::vector<int> n = GetNeighbors( queue[i] );
			double dist = mNodes[ queue[i] ].obsDist;

			for( int j = 0; j < n.size(); ++j ) {
				double new_dist = dist + EdgeCost( queue[i], n[j] );
				if( new_dist < mNodes[ n[j] ].obsDist ) {
					mNodes[ n[j] ].obsDist = new_dist;
					newQueue.push_back( n[j] );
				}
			}
		}

		queue.clear();
		queue = newQueue;
		newQueue.clear();

	}
	printf("--) Finishing calculating DT \n");


}

/**
 * @function GetDTRidge
 */
std::vector<Node3D*> BS::GetDTRidge() {

	std::vector<Node3D*> ridge;
	
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {
			for( int k = 0; k < mSizeZ; ++k ) {
				int x = ref( i, j, k );
				if( GetState( i, j, k ) != OBSTACLE_STATE && IsLocalMaxima( x ) == true ) {							
					ridge.push_back( &mNodes[x] );
				}		
			}		
		}
	}	

	return ridge;

}

/**
 * @function IsLocalMaxima
 */
bool BS::IsLocalMaxima( int _index ) {

	Node3D node = mNodes[ _index ];
    float dist = node.obsDist;		
	std::vector<int> nx = GetNeighbors( _index );

	for( int i = 0; i < nx.size(); ++i ) {

		Node3D neigh = mNodes[ nx[i] ];

		if( EdgeCost( nx[i], _index ) == 1.0 && neigh.obsDist >= node.obsDist + 1.0  ) {
			return false;		
		}
		else if( EdgeCost( nx[i], _index ) == BS_SQRT2 && neigh.obsDist >= node.obsDist  + BS_SQRT2 ) { // BS_SQRT2 // 0.95		
			return false;
		}
		else if( EdgeCost( nx[i], _index ) == BS_SQRT3 && neigh.obsDist >= node.obsDist  + BS_SQRT3 ) {		
			return false;
		}
	}

	return true;

}

////////////////// Visualization Functions //////////////////

/**
 * @function ViewObstacles
 */
void BS::ViewObstacles( pcl::visualization::PCLVisualizer *_viewer,
					    int _r, int _g, int _b ) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr obstaclesCloud( new pcl::PointCloud<pcl::PointXYZ> );

	obstaclesCloud->height = 1;
	obstaclesCloud->is_dense = false;
	obstaclesCloud->points.resize( 0 );

	pcl::PointXYZ q;
		
	for( size_t i = 0; i < mSizeX; ++i ) {	
		for( size_t j = 0; j < mSizeY; ++j ) {
			for( size_t k = 0; k < mSizeZ; ++k ) {
				if( GetState( i, j, k ) == OBSTACLE_STATE ) { // Obstacle
					q.x = (double) i; q.y = (double) j; q.z = (double)k;				
					obstaclesCloud->points.push_back(q);       
				}
			}
		}
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> obstaclesColor( obstaclesCloud, _r, _g, _b );

	_viewer->addPointCloud<pcl::PointXYZ>( obstaclesCloud, obstaclesColor, "Obstacles Cloud" );
	
}

/**
 * @function ViewVoronoi
 */
void BS::ViewVoronoi( pcl::visualization::PCLVisualizer *_viewer,
	   	  		      int _r, int _g, int _b ) {

	std::vector<Node3D*> ridge = GetDTRidge();

	pcl::PointCloud<pcl::PointXYZ>::Ptr ridgeCloud( new pcl::PointCloud<pcl::PointXYZ> );

	ridgeCloud->height = 1;
	ridgeCloud->is_dense = false;
	ridgeCloud->points.resize( 0 );
		
	for( size_t i = 0; i < ridge.size() ; ++i ) {

		pcl::PointXYZ q;
		q.x = ridge[i]->x; 
		q.y = ridge[i]->y; 
		q.z = ridge[i]->z;					
		ridgeCloud->points.push_back(q);       
	}

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ridgeColor( ridgeCloud, _r, _g, _b );

	_viewer->addPointCloud<pcl::PointXYZ>( ridgeCloud, ridgeColor, "Ridge Cloud" );

}

///////////// Geometry Helper Functions ////////////////////

/**
 * @function CreateExternalBoundary
 * @brief Create a wall around the available space
 */
void BS::CreateExternalBoundary() {

	//-- Add the faces
	for( size_t j = 0; j < mSizeY; ++j ) {
		for( size_t k = 0; k < mSizeZ; ++k ) {
			SetState( 0, j, k, OBSTACLE_STATE );			// Face 2
			SetState( mSizeX - 1, j, k, OBSTACLE_STATE );	// Face 4
		}	
	}

	for( size_t i = 0; i < mSizeX; ++i ) {
		for( size_t k = 0; k < mSizeZ; ++k ) {
			SetState( i, 0, k, OBSTACLE_STATE );			// Face 1
			SetState( i, mSizeY - 1, k, OBSTACLE_STATE );	// Face 3
		}	
	}

	for( size_t i = 0; i < mSizeX; ++i ) {
		for( size_t j = 0; j < mSizeY; ++j ) {
			SetState( i, j, 0, OBSTACLE_STATE );			// Face 6
			SetState( i, j, mSizeZ - 1, OBSTACLE_STATE );	// Face 5
		}	
	}
}

/**
 * @function CreateBox
 */
void BS::CreateBox( int _x, int _y, int _z, int _dimX, int _dimY, int _dimZ ) {

	for( size_t i = _x; i < _x + _dimX; ++i ) {
		for( size_t j = _y; j < _y + _dimY; ++j ) {
			for( size_t k = _z; k < _z + _dimZ; ++k ) {
				SetState( i, j, k, OBSTACLE_STATE );
			}
		}
	}
}

