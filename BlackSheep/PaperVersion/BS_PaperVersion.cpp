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

#include "BS_PaperVersion.h"
#include <cfloat>
#include <cmath>

const float BS::BS_INF = FLT_MAX;
const float BS::BS_SQRT2 = 1.414;
const float BS::BS_SQRT3 = 1.732;	

const float BS::sNominalValue = 1.0;
const float BS::sDefaultBrushDist = 1.0;

const int BS::sMaxIter = 500000;

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
	mGeometricNeighbors = new std::vector<int>[mNumNodes];

	mCountPaths = 0;

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
	if( mHT != NULL ) {
		delete [] mHT;
	}

	if( mGeometricNeighbors != NULL ) {
		delete [] mGeometricNeighbors;
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
 * @function GetState
 */
int BS::GetState( int _ref ) const{
	return mNodes[_ref].state;
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
float BS::EdgeCost( int _index1, int _index2, float _value ) {

	Node3D n1 = mNodes[_index1];
	Node3D n2 = mNodes[_index2];

    int d = abs( n1.x - n2.x ) + abs( n1.y - n2.y ) + abs( n1.z - n2.z );

	if( d == 3 ) {
		return BS_SQRT3*_value; // sqrt(3)
	}

	else if( d == 2 ) {
		return BS_SQRT2*_value; // sqrt(2)
	}
	else if( d == 1 ) {
		return 1.0*_value; // 1
	}
	else {
		printf( "--> WTH. There is an error here! d: %d \n", d );
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

	Node3D *node = &mNodes[_ref];
	int x = node->x;
	int y = node->y;
	int z = node->z;
	int nx; int ny; int nz;

	for( int i = 0; i < 26; ++i ) {
		nx = x + NX[i];
		ny = y + NY[i];
		nz = z + NZ[i];

		if( IsValid( nx, ny, nz ) == true && GetState( nx, ny, nz) == FREE_STATE ) {
			neighbors.push_back( ref(nx, ny, nz) );
		}
	}		
	return neighbors;
}

/**
 * @function CalculateDT
 */
void BS::CalculateDT() {

	printf("Getting Geometric Neighbors \n");
	for( size_t i = 0; i < mNumNodes; ++i ) {
		mGeometricNeighbors[i] = GetNeighbors(i);
	}
	printf("End of Geometric Neighbors \n");


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
			std::vector<int> n = mGeometricNeighbors[ queue[i] ];
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

/////////////////// Search Functions ////////////////////////

/**
 * @function InitSearch
 */
void BS::InitSearch() {

	printf("Start Init Search \n");
	//-- Reset the Hash Table for OpenSet and OpenSet
	mHT = new int[mNumNodes];
	std::fill( mHT, mHT + mNumNodes, -1 );

	mOpenSet.resize(0);
	mMaxBrushDist = sDefaultBrushDist;	

	//-- Reset the nodes
	Node3D *node;
	for( size_t i = 0; i < mNumNodes; ++i ) {
	
		node = &mNodes[i];	

		node->s.costF = BS_INF;
		node->s.costG = BS_INF;
		node->s.costH = BS_INF;
		node->s.value = 0; // For the first search
		node->s.brushDist = sDefaultBrushDist; // Initially all have the same brushDist for the first search
		node->s.status = IN_NO_SET;
		node->s.parent = -1;
	}

	printf("Finishing Init Search \n");
}

/**
 * @function FindVarietyPaths
 */
std::vector< std::vector<Eigen::Vector3i> > BS::FindVarietyPaths( int _x1, int _y1, int _z1,
									   			   	  		      int _x2, int _y2, int _z2, int _times, float _alpha ) {

	mAlpha = _alpha;
	std::vector< std::vector<Eigen::Vector3i> > paths;
    std::vector<Eigen::Vector3i> path;
	std::vector< std::vector<int> > nodePaths;
	std::vector<int> allNodePaths;

	InitSearch();

	for( size_t i = 0; i < _times; ++i ) {
		path.resize(0);
		path = FindPath( _x1, _y1, _z1, _x2, _y2, _z2 );
		paths.push_back( path );
		nodePaths.push_back( mPath );
		//-- Update the values
		allNodePaths = JoinPaths( nodePaths );
		printf("allNodePaths size: %d \n", allNodePaths.size() );
		UpdateNodeValues( allNodePaths );
		// Reset the search
		ResetSearch();
	}

	return paths;
}

/**
 * @function CalculateDTPaths
 */
void BS::CalculateDTPaths( std::vector<int> _path ) {

	printf("--) Calculating DT Paths \n");
	std::vector<int> queue = _path;

	//-- 1. Initialize queue with _path nodes and set distances to zero for them
	for( int i = 0; i < mNumNodes; ++i ) {
		if( mNodes[i].state == FREE_STATE ) {
			mNodes[i].s.brushDist = BS_INF;
		} else {
			mNodes[i].s.brushDist = 0;
		}
	}

	// Path nodes with zero initial value
	for( int i = 0; i < _path.size(); ++i ) {
		mNodes[ _path[i] ].s.brushDist = 0;
	}
	

	//-- 2. Loop until no more new elements in Queue
	std::vector<int> newQueue(0);

	while( queue.size() > 0 ) {

		for( int i = 0; i < queue.size(); ++i ) {
			std::vector<int> n = mGeometricNeighbors[ queue[i] ];
			double dist = mNodes[ queue[i] ].s.brushDist;

			for( int j = 0; j < n.size(); ++j ) {
				double new_dist = dist + EdgeCost( queue[i], n[j], sNominalValue );
				if( new_dist < mNodes[ n[j] ].s.brushDist ) {
					mNodes[ n[j] ].s.brushDist = new_dist;
					newQueue.push_back( n[j] );
				}
			}
		}

		queue.clear();
		queue = newQueue;
		newQueue.clear();

	}
	printf("--) Finishing calculating DT Paths \n");
}

/**
 * @function UpdateNodeValues
 */
void BS::UpdateNodeValues( std::vector<int> _path ) {
	CalculateDTPaths(_path );	

	//-- Our DT is ready. Let's check the minimum and maximum guys
	float minBrushDist = BS_INF;
	float maxBrushDist = -BS_INF;

	for( int i = 0; i < mNumNodes; ++i ) {
		if( mNodes[i].s.brushDist < minBrushDist ) {
			minBrushDist = mNodes[i].s.brushDist;
		}

		if( mNodes[i].s.brushDist > maxBrushDist ) {
			maxBrushDist = mNodes[i].s.brushDist;
		}
		
	}

	mMaxBrushDist = maxBrushDist;

	printf("--(i) Min and max: %f , %f \n", minBrushDist, maxBrushDist );


	//-- Update accordingly
	for( int i = 0; i < mNumNodes; ++i ) {
		//mNodes[i].s.value = ( maxBrushDist - mNodes[i].s.brushDist );
        mNodes[i].s.value = ( maxBrushDist - mNodes[i].s.brushDist )/maxBrushDist;
	}

}

/**
 * @function JoinPaths
 * @brief FIXME CUT OFF REPEATED GUYS	
 */
std::vector<int> BS::JoinPaths( std::vector< std::vector<int> >  _allPaths ) {

	std::vector<int> bunchPaths;

	for( int i = 0; i < _allPaths.size(); ++i ) {
		for( int j = 0; j < _allPaths[i].size(); ++j ) {
			bunchPaths.push_back( _allPaths[i][j] );			
		}
	}

	return bunchPaths;
}

/**
 * @function ResetSearch 
 */
void BS::ResetSearch() {

	for( int i = 0; i < mNumNodes; ++i ) {
		mNodes[i].s.costF = BS_INF;
		mNodes[i].s.costG = BS_INF;
		mNodes[i].s.costH = BS_INF; 
		mNodes[i].s.status = IN_NO_SET;
	}

	mOpenSet.resize(0);
}

/**
 * @function FindPath
 */
std::vector<Eigen::Vector3i> BS::FindPath( int _x1, int _y1, int _z1,
										   int _x2, int _y2, int _z2 ) {

	std::vector<Eigen::Vector3i> path;
	printf( "Start Search A* \n" );

	if( IsValid(_x1, _y1, _z1) == false || IsValid(_x2, _y2, _z2) == false ) {
		printf("Error, Start or Target NO valid, Exiting \n" ); return path;
	}

	int nodeStart = ref( _x1, _y1, _z1 );
	int nodeTarget = ref( _x2, _y2, _z2 );


	//-- Fill start space
	Node3D *node;
	node = &mNodes[nodeStart];
	node->s.costG = 0;
	node->s.costH = CostHeuristic( nodeStart, nodeTarget )*( 0 + mAlpha );
	node->s.costF = node->s.costG + node->s.costH;
	node->s.parent = -1;
	node->s.status = IN_NO_SET;

	//-- Push it into the open set
	PushOpenSet( nodeStart );

	//-- Loop
	int x;
	int count = 0;

	while( count < sMaxIter ) {
		count++;
		//-- Remove top node in OpenSet
		try {
			x = PopOpenSet();
		} catch( int i ) {
			printf( "-- (%d) No more nodes to pop out \n", i );
			break;
		}

		//-- Check if it is goal
		if( x == nodeTarget ) {
			printf( "--> Found a path! \n" ); TracePath( x, path );break;
		}
		
		//-- Add node to closed set
		mNodes[x].s.status = IN_CLOSED_SET;
		std::vector<int> neighbors = GetNeighbors( x );

		//-- 
		for( int i = 0; i < neighbors.size(); i++ ) {
			if( mNodes[ neighbors[i] ].s.status == IN_CLOSED_SET ) {
				continue;	
			}
			
			int y = mNodes[ neighbors[i] ].index; // Same as neighbors[i] actually
			float tentative_G_score = mNodes[x].s.costG + EdgeCost( x, y, ( mNodes[y].s.value + mAlpha) );
			
			if( mNodes[y].s.status != IN_OPEN_SET ) {
				node = &mNodes[y];
				node->s.parent = x;
				node->s.costG = tentative_G_score;
				node->s.costH = CostHeuristic( y, nodeTarget )*( 0 + mAlpha );
				node->s.costF = node->s.costG + node->s.costH;
				PushOpenSet(y);
			}
			else {
				if( tentative_G_score < mNodes[y].s.costG ) {
					node = &mNodes[y];
					node->s.parent = x;
					node->s.costG = tentative_G_score;
					node->s.costH = CostHeuristic( y, nodeTarget )*( 0 + mAlpha );
					node->s.costF = node->s.costG + node->s.costH;
					//-- Reorder your OpenSet
					UpdateLowerOpenSet( y );				
				}
			}
		} //-- End for every neighbor

		
	} //-- End of while
	node = NULL;
	printf("I am done \n");
	return path;
}

/**
 * @function CostHeuristic
 */
float BS::CostHeuristic( int _start, int _target ) {

	Node3D nStart= mNodes[_start];
	Node3D nTarget = mNodes[_target];

	float dx = nStart.x - nTarget.x;
	float dy = nStart.y - nTarget.y;
	float dz = nStart.z - nTarget.z;

	return sqrt( dx*dx + dy*dy + dz*dz );
}

/**
 * @function PushOpenSet
 */
void BS::PushOpenSet( int _key ) {

	int n; 
    int node; int parent;
    int temp;

    //-- Sign the flag
    mNodes[_key].s.status = IN_OPEN_SET;

    mOpenSet.push_back( _key );
    n = mOpenSet.size() - 1;
 
    // If this is the first element added
    if( n == 0 ) { 
  	    mHT[_key] = n;
	    return; 
	}

    // If not, start on the bottom and go up
    node = n;

    int qp; int qn;

    while( node != 0 )
    {
    	parent = floor( (node - 1)/2 );
		qp = mOpenSet[parent];
    	qn = mOpenSet[node];
    	// Always try to put new nodes up
    	if( mNodes[qp].s.costF >= mNodes[qn].s.costF )
      	{
        	temp = mOpenSet[parent];
        	mOpenSet[parent] = qn; mHT[qn] = parent; 
        	mOpenSet[node] = temp; mHT[temp] = node; 
        	node = parent; 
      	}  
    	else
     	{ break; }
    }   

 	mHT[_key] = node;

}

/**
 * @function PopOpenSet
 */
int BS::PopOpenSet() { 

  int first; int bottom;
  int node;
  int child_1; int child_2;
  int n; 
  int temp;

  if( mOpenSet.size() == 0 )
    { throw -1; }

  // Save the pop-out element
  first = mOpenSet[0];
  
  // Reorder your binary heap
  bottom = mOpenSet.size() - 1;

  mOpenSet[0] = mOpenSet[bottom]; mHT[ mOpenSet[bottom] ] = 0;
  mOpenSet.pop_back();
  n = mOpenSet.size();

  int u = 0;

  int qu;
  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( mNodes[ mOpenSet[node] ].s.costF >= mNodes[ mOpenSet[child_1] ].s.costF )
        { u = child_1;  }
       if( mNodes[ mOpenSet[u] ].s.costF  >= mNodes[ mOpenSet[child_2] ].s.costF )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( mNodes[ mOpenSet[node] ].s.costF >= mNodes[ mOpenSet[child_1] ].s.costF )
         { u = child_1; }
     }
    
	qu = mOpenSet[u];
    if( node != u )
     { temp = mOpenSet[node];
       mOpenSet[node] = qu; mHT[qu] = node;
       mOpenSet[u] = temp; mHT[temp] = u; 
     }

    else
     { break; } 
  }

  return first;

}

/**
 * @function UpdateLowerOpenSet
 * @brief Update a node with a lower value
 */
void BS::UpdateLowerOpenSet( int _key ) { 
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  n = mHT[_key];

  //-- If it happens to be the first element
  if( n == 0 )
    { return; } //-- mHT[ID] = 0; // the same

  //-- If not, start on the bottom and go up
  node = n;

  int qp; int qn;

  while( node != 0 )
  { 
    parent = floor( (node - 1)/2 );

	qp = mOpenSet[parent]; qn = mOpenSet[node];
    // Always try to put new nodes up
    if( mNodes[ qp ].s.costF > mNodes[ qn ].s.costF )
      {
        temp = mOpenSet[parent];
        mOpenSet[parent] = qn; mHT[qn] = parent;
        mOpenSet[node] = temp; mHT[temp] = node; 
        node = parent; 
      }  
    else
     {  break; }
  }   

}



/**
 * @function TracePath
 * @brief Trace path, literally
 */
bool BS::TracePath( const int &_key, std::vector<Eigen::Vector3i> & _path  )
{
  printf("--> Trace path \n");
  std::vector<int> backPath(0);

  mPath.resize(0);
  _path.resize(0);

  int n = _key;

  while( n != -1 ) 
  {
    backPath.push_back( n );
    n = mNodes[n].s.parent;
  }

  int b = backPath.size();

  Node3D *node;
  for( int i = 0; i < b; i++ )
     { 
	   node = &mNodes[ backPath[ b - 1- i] ];
	   Eigen::Vector3i p; p << node->x, node->y, node->z; 
	   _path.push_back( p );
	   mPath.push_back( backPath[b-1-i] );
	 }
    node = NULL;
  if( _path.size() > 0 )
    { return true; }

  return false;  
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

/**
 * @function ViewPath
 */
void BS::ViewPath( std::vector<Eigen::Vector3i> _path, pcl::visualization::PCLVisualizer *_viewer,
				   int _r, int _g, int _b ) {

	printf( "Plotting a path of %d points \n", _path.size() );

	pcl::PointXYZ q;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pathCloud( new pcl::PointCloud<pcl::PointXYZ> );
	for( int j = 0; j < _path.size(); ++j ) {
		q.x = _path[j](0);
		q.y = _path[j](1);
		q.z = _path[j](2);
		pathCloud->points.push_back(q);
	}  		
	
	double r; double g; double b;
	r = ( _r % 256 )/255.0;
	g = ( _g % 256 )/255.0;
	b = ( _b % 256 )/255.0;

	for( int j = 0; j < pathCloud->points.size() - 1; ++j ) {
		char linename[15];	
		sprintf( linename, "path%d-%d", mCountPaths, j );
		std::string id(linename);
		_viewer->addLine<pcl::PointXYZ>( pathCloud->points[j], pathCloud->points[j + 1], r, g, b, id );
	}

	mCountPaths++;
}

/**
 * @function ViewPaths
 */				   
void BS::ViewPaths( std::vector< std::vector<Eigen::Vector3i> > _paths, pcl::visualization::PCLVisualizer *_viewer ) {

	srand( time(NULL) );

	int numPaths = _paths.size();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pathCloud;

	pcl::PointXYZ q;
	for( size_t i = 0; i < numPaths; ++i ) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr pc( new pcl::PointCloud<pcl::PointXYZ> );
		for( int j = 0; j < _paths[i].size(); ++j ) {
			q.x = _paths[i][j](0);
			q.y = _paths[i][j](1);
			q.z = _paths[i][j](2);
			pc->points.push_back(q);
		}  		
		pathCloud.push_back( pc );
	}


	for( size_t i = 0; i < numPaths; ++i ) {
		double r; double g; double b;
		r = ( rand() % 256 )/255.0;
		g = ( rand() % 256 )/255.0;
		b = ( rand() % 256 )/255.0;

		for( int j = 0; j < pathCloud[i]->points.size() - 1; ++j ) {
			char linename[16];	
			sprintf( linename, "paths%d-%d", i,j );
			std::string id(linename);
			_viewer->addLine<pcl::PointXYZ>( pathCloud[i]->points[j], pathCloud[i]->points[j + 1], r, g, b, id );
		}
	}
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

