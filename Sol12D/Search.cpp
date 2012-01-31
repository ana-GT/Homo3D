/**
 * @file Search.cpp
 */

#include "Search.h"
#include <stdint.h>
#include <stdio.h>

const int Search::sMaxIter = 50000;
const int Search::sNominalValue = 1.0;
const double Search::SEARCH_INF = 10000;

/**
 * @function Search
 */
Search::Search( std::vector<Cell> *_cells ) {

	mNumCells = _cells->size(); 
	mCellNeighborRange = 1.5; // bigger than sqrt(2), smaller than 2
	mEpsilon =1;

	//-- 2. Add points to kd-tree and create the nodes
	mKdTree = kd_create( 2 ); // 2-dimensional space

	for( int i = 0; i < mNumCells; ++i ) { 
		uintptr_t id = i; 
		double x = (double) (*_cells)[i].pos.x;
		double y = (double) (*_cells)[i].pos.y;

		Eigen::VectorXd entity(2); 
		entity(0) = x; entity(1) = y;
		kd_insert( mKdTree, entity.data(), (void*)id );

	    Node node; 
		node.index = i; 
		node.x = x; node.y = y;
		node.status = IN_NO_SET;
		node.value = sNominalValue;
		node.costF = SEARCH_INF;
		node.costG = SEARCH_INF;
		node.costH = SEARCH_INF;
		node.parent = -1;
	
	    mNodes.push_back( node );
	}
	
	//-- Fill the neighbors
	for( int i = 0; i < mNumCells; ++i ) {
	
		Eigen::VectorXd entity(2); 
		entity(0) = mNodes[i].x; 
		entity(1) = mNodes[i].y; 

		struct kdres* neighset = kd_nearest_range( mKdTree, entity.data(), (double)mCellNeighborRange );

		int size = kd_res_size( neighset );

		mNodes[i].neighbors.resize(0);

		for( int j = 0; j < size; ++j ) {
		    uintptr_t near = (uintptr_t) kd_res_item_data( neighset ); 
			if( mNodes[i].index != near ) { 
				mNodes[i].neighbors.push_back( near );
			}
			kd_res_next( neighset );			
		}

		if( mNodes[i].neighbors.size() == 0 || mNodes[i].neighbors.size() > 8 ) {
			printf("Error filling neighbors. Either too few or too much \n");
		}

		//-- Free the structure
		kd_res_free( neighset );
	}
} 

/**
 * @function Search
 * @brief Reset the costs but NOT the brushDistance neither the values
 */
Search::~Search() {
	kd_free( mKdTree );
}


/**
 * @function FindDiversePaths
 */
std::vector<std::vector<Pos> > Search::FindDiversePaths( double _startX, double _startY, double _goalX, double _goalY, int _times ) {

	std::vector< std::vector<Pos> >  diversePaths;
	std::vector<Pos> path;

	for( int i = 0; i < _times; i++ ) {
		path.resize(0);
		path = FindPath( _startX, _startY, _goalX, _goalY );
		diversePaths.push_back( path );
		// Update the values
		UpdateNodeValues( mPath );
		// Reset the search
		ResetSearch();
	}
	
	return diversePaths;
}

/**
 * @functio ResetSearch 
 */
void Search::ResetSearch() {

	for( int i = 0; i < mNumCells; ++i ) {
		mNodes[i].status = IN_NO_SET;
		mNodes[i].costF = 0;
		mNodes[i].costG = 0;
		mNodes[i].costH = 0;
	}

	mOpenSet.resize(0);
}

/**
 * @function UpdateNodeValues
 */
void Search::UpdateNodeValues( std::vector<int> _path ) {

	printf(" Updating values \n" );
	std::vector<int> queue = _path;
	
	//-- 1. Initialize queue with _path nodes and set distances to zero for them
	for( int i = 0; i < mNumCells; ++i ) {
		mNodes[i].brushDistance = SEARCH_INF;
	}

	// Path nodes with zero initial value
	for( int i = 0; i < _path.size(); ++i ) {
		mNodes[ _path[i] ].brushDistance = 0;
	}


	//-- 2. Loop until no more new elements in Queue
	std::vector<int> newQueue(0);

	while( queue.size() > 0 ) {

		for( int i = 0; i < queue.size(); ++i ) {

			double dist = mNodes[ queue[i] ].brushDistance;
			std::vector<int> n = mNodes[ queue[i] ].neighbors;

			for( int j = 0; j < n.size(); ++j ) {
				double new_dist = dist + edgeCost( queue[i], n[j], sNominalValue );
				if( new_dist < mNodes[ n[j] ].brushDistance ) {
					mNodes[ n[j] ].brushDistance = new_dist;
					newQueue.push_back( n[j] );
				}
			}
		}

		queue.clear();
		queue = newQueue;
		newQueue.clear();

	}

	//-- 3. Our DT is ready. Let's check the minimum and maximum guys
	double minBrushDist = 1000;
	double maxBrushDist = -1000;

	for( int i = 0; i < mNumCells; ++i ) {
		if( mNodes[i].brushDistance < minBrushDist ) {
			minBrushDist = mNodes[i].brushDistance;
		}

		if( mNodes[i].brushDistance > maxBrushDist ) {
			maxBrushDist = mNodes[i].brushDistance;
		}
		
	}

	printf("Min and max: %f , %f \n", minBrushDist, maxBrushDist );


	//-- Add accordingly
	for( int i = 0; i < mNumCells; ++i ) {
		mNodes[i].value += ( maxBrushDist - mNodes[i].brushDistance );
	}

}

/**
 * @function FindPath
 */
std::vector<Pos> Search::FindPath( double _startX, double _startY, double _goalX, double _goalY ) {
	
	int startCell; int goalCell;

	//-- Search
	for( int i = 0; i < mNumCells; ++i ) {
		if( _startX == mNodes[i].x  && _startY == mNodes[i].y  ) {
			startCell = i; break; 
		}
	}

	for( int i = 0; i < mNumCells; ++i ) {
		if( _goalX == mNodes[i].x  && _goalY == mNodes[i].y ) {
			goalCell = i; break;
		}
	}

	printf( "-- Start search A* \n" );
	
	//-- Fill the starting state
	mNodes[startCell].costG = 0;
	mNodes[startCell].costH = costHeuristic( startCell, goalCell );
	mNodes[startCell].costF = mNodes[startCell].costG + mEpsilon*mNodes[startCell].costH;
	mNodes[startCell].parent = -1;
	mNodes[startCell].status = IN_NO_SET;
		
	//-- Push it into the open set
	pushOpenSet( startCell );
	
	//-- Loop
	int x;
	int count = 0;

	std::vector<Pos> path;

	while( count < sMaxIter ) {
		count++;
		//-- Remove top node in OpenSet
		try {
			x = popOpenSet();
		} catch( int i ) {
			printf( "-- (%d) No more nodes to pop out \n", i );
			break;
		}

		//-- Check if it is goal
		if( x == goalCell ) {
		
			printf( "--> Found a path! \n" ); tracePath( x, path ); break;
		}
		
		//-- Add node to closed set
		mNodes[x].status = IN_CLOSED_SET;
		std::vector<int> neighbors = mNodes[x].neighbors;

		//-- 
		for( int i = 0; i < neighbors.size(); i++ ) {
			if( mNodes[ neighbors[i] ].status == IN_CLOSED_SET ) {
				continue;	
			}
			
			int y = mNodes[ neighbors[i] ].index; // Same as neighbors[i] actually
			double tentative_G_score = mNodes[x].costG + edgeCost( x, y, mNodes[y].value );
			
			if( mNodes[y].status != IN_OPEN_SET ) {
				mNodes[y].parent = x;
				mNodes[y].costG = tentative_G_score;
				mNodes[y].costH = costHeuristic( y, goalCell );
				mNodes[y].costF = mNodes[y].costG + mEpsilon*mNodes[y].costH;
				pushOpenSet(y);
			}
			else {
				if( tentative_G_score < mNodes[y].costG ) {
					mNodes[y].parent = x;
					mNodes[y].costG = tentative_G_score;
					mNodes[y].costH = costHeuristic( y, goalCell );
					mNodes[y].costF = mNodes[y].costG + mEpsilon*mNodes[y].costH;
					//-- Reorder your OpenSet
					updateLowerOpenSet( y );				
				}
			}
		} //-- End for every neighbor

		
	} //-- End of while

	printf("I am done \n");
	return path;
		
}

/**
 * @function costHeuristic
 * @brief Indices from mCell
 */
double Search::costHeuristic( int _start, int _goal ) {

	//-- Base cost for a node: 1.0

	double sx = mNodes[_start].x;
	double sy = mNodes[_start].y;

	double gx = mNodes[_goal].x;
	double gy = mNodes[_goal].y;

	double dx = sx - gx;
	double dy = sy - gy;

	return sqrt( dx*dx + dy*dy )*sNominalValue;

}

/** 
 * @function edgeCost
 */
double Search::edgeCost( int _nodeFrom, int _nodeTo, double _value ) {

	double fromX = mNodes[_nodeFrom].x;
	double fromY = mNodes[_nodeFrom].y;

	double toX = mNodes[_nodeTo].x;
	double toY = mNodes[_nodeTo].y;

	double dX = abs( fromX - toX );
	double dY = abs( fromY - toY );

	double sum = dX + dY;
	if( sum == 1.0 ) {
		return 1.0*_value;
	}	
	if( sum == 2.0 ) {
		return sqrt(2)*_value;
	}
	else {printf("[edgeCost] What the heck? ERROR in getting neighbors!!!!!!  sum: %f  %d(%f %f) %d(%f, %f) \n", sum, _nodeFrom, fromX, fromY, _nodeTo,toX, toY); return 1000;}
}



/**
 * @function pushOpenSet
 * @brief push the node ID into the openSet binary heap
 */
void Search::pushOpenSet( int _key ) { 

  int n; 
  int node; int parent;
  int temp;

  //-- Sign the flag
  mNodes[_key].status = IN_OPEN_SET;

  mOpenSet.push_back( _key );
  n = mOpenSet.size() - 1;

  // If this is the first element added
  if( n == 0 )
    { return; }

  // If not, start on the bottom and go up
  node = n;

  while( node != 0 )
  {
    parent = floor( (node - 1)/2 );
    // Always try to put new nodes up
    if( mNodes[ mOpenSet[parent] ].costF >= mNodes[ mOpenSet[node] ].costF )
      {
        temp = mOpenSet[parent];
        mOpenSet[parent] = mOpenSet[node];
        mOpenSet[node] = temp; 
        node = parent; 
      }  
    else
     { break; }
  }   

}

/**
 * @function popOpenSet
 */
int Search::popOpenSet() { 

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

  mOpenSet[0] = mOpenSet[bottom];
  mOpenSet.pop_back();
  n = mOpenSet.size();

  int u = 0;

  while( true )
  {
    node = u;

    child_1 = 2*node + 1;
    child_2 = 2*node + 2; 

    if( child_2 < n )
     {  
       if( mNodes[ mOpenSet[node] ].costF >= mNodes[ mOpenSet[child_1] ].costF )
        { u = child_1;  }
       if( mNodes[ mOpenSet[u] ].costF  >= mNodes[ mOpenSet[child_2] ].costF )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( mNodes[ mOpenSet[node] ].costF >= mNodes[ mOpenSet[child_1] ].costF )
         { u = child_1; }
     }
    
    if( node != u )
     { temp = mOpenSet[node];
       mOpenSet[node] = mOpenSet[u];
       mOpenSet[u] = temp; 
     }

    else
     { break; } 
  }

  return first;

}

/**
 * @function updateLowerOpenSet
 * @brief Update a node with a lower value
 */
void Search::updateLowerOpenSet( int key ) { 
  int n; 
  int node; int parent;
  int temp;

  //-- Find your guy
  for( int i = 0; i < mOpenSet.size(); i++ )
  {
    if( mOpenSet[i] == key )
    { n = i; break; }
  }

  //printf(" Start pos: %d f: %f \n ", n, nodes_[ openSet_[n] ].f_score );
  //-- If it happens to be the first element
  if( n == 0 )
    { return; }

  //-- If not, start on the bottom and go up
  node = n;

  while( node != 0 )
  { 
    parent = floor( (node - 1)/2 );
    // Always try to put new nodes up
    if( mNodes[ mOpenSet[parent] ].costF > mNodes[ mOpenSet[node] ].costF )
      {
        //printf(" Parent pos: %d f: %f \n ", parent, nodes_[ openSet_[parent] ].f_score );
        temp = mOpenSet[parent];
        mOpenSet[parent] = mOpenSet[node];
        mOpenSet[node] = temp; 
        node = parent; 
      }  
    else
     {  // printf(" End pos: %d f: %f \n ", node, nodes_[ openSet_[node] ].f_score ); 
        break; }
  }   

}



/**
 * @function tracePath
 * @brief Trace path, literally
 */
bool Search::tracePath( const int &key, std::vector<Pos> & _posPath  )
{
  printf("Trace path \n");
  std::vector<int> backPath(0);

  mPath.resize(0); _posPath.resize(0);

  int n = key;

  while( n != -1 ) 
  {
    backPath.push_back( n );
    n = mNodes[n].parent;
  }

  int b = backPath.size();

  for( int i = 0; i < b; i++ )
     { mPath.push_back( backPath[ b - 1- i] );
	   Pos p;
	   p.x = (int) mNodes[mPath[i] ].x;
	   p.y = (int) mNodes[mPath[i] ].y;
	   _posPath.push_back( p ); 
	 }

  if( _posPath.size() > 0 )
    { return true; }

  return false;  
}  

/**
 * @function CheckNeighbors
 */
void Search::CheckNeighbors( double _x, double _y ) {

	//-- Search
	int cell;

	for( int i = 0; i < mNumCells; ++i ) {
		if( _x == mNodes[i].x  && _y == mNodes[i].y  ) {
			cell = i; break; 
		}
	}	

	std::vector<int> n = mNodes[cell].neighbors;
	printf(" ** Neighbors of (%d, %d) \n", (int)mNodes[cell].x, (int)mNodes[cell].y );
	for( int i = 0; i < n.size(); ++i ) {
		printf(" [%d] (%d, %d) \n", i, (int)mNodes[n[i]].x, (int)mNodes[n[i]].y );
	}
}

