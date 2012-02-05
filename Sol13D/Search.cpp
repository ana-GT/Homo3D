/**
 * @file Search.cpp
 */

#include "Search.h"
#include <stdint.h>
#include <stdio.h>
#include <cfloat>

const int Search::sMaxIter = 500000;
const double Search::sNominalValue = 1.0;
const double Search::SEARCH_INF = DBL_MAX;

/**
 * @function Search
 */
Search::Search( std::vector<Cell> *_cells ) {

	mNumNodes = _cells->size(); 
	mNodeNeighborRange = 1.8; // bigger than sqrt(3), smaller than 2
	mEpsilon = 1;

	//-- 2. Add points to kd-tree and create the nodes
	mKdTree = kd_create( 3 ); // 2-dimensional space

	mNodes = new Node[mNumNodes];
	HT = new int[mNumNodes];
	std::fill( HT, HT + mNumNodes, -1 );

	for( int i = 0; i < mNumNodes; ++i ) { 
		uintptr_t id = i; 
		Pos p = (*_cells)[i].pos;

		Eigen::VectorXd entity(3); 
		entity<<  (double)p.x, (double)p.y, (double)p.z;
		kd_insert( mKdTree, entity.data(), (void*)id );

	    Node node; 
		node.index = i; 
		node.pos = p;
		node.status = IN_NO_SET;
		node.value = sNominalValue;
		node.cost.F = SEARCH_INF;
		node.cost.G = SEARCH_INF;
		node.cost.H = SEARCH_INF;
		node.parent = -1;
	
	    mNodes[i] = node;
	}
	
	//-- Fill the neighbors
	for( int i = 0; i < mNumNodes; ++i ) {
	
		Eigen::VectorXd entity(3); 
		Pos p = mNodes[i].pos;
		entity<< (double)p.x, (double)p.y, (double)p.z;

		struct kdres* neighset = kd_nearest_range( mKdTree, entity.data(), (double)mNodeNeighborRange );

		int size = kd_res_size( neighset );

		mNodes[i].neighbors.resize(0);

		for( int j = 0; j < size; ++j ) {
		    uintptr_t near = (uintptr_t) kd_res_item_data( neighset ); 
			if( mNodes[i].index != near ) { 
				mNodes[i].neighbors.push_back( near );
			}
			kd_res_next( neighset );			
		}

		size = mNodes[i].neighbors.size();
		if( size == 0 || size > 26 ) {
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
	if( mNodes != NULL ) {
		delete [] mNodes;
	}
	if( HT != NULL ) {
		delete [] HT;
	}

}


/**
 * @function FindDiversePaths
 */
std::vector<std::vector<Pos> > Search::FindDiversePaths(  Pos _start, Pos _goal, int _times ) {

	std::vector< std::vector<Pos> >  diversePaths;
	std::vector<Pos> path;
	std::vector< std::vector<int> > nodePaths;
	std::vector<int> allNodePaths;

	for( int i = 0; i < _times; i++ ) {
		path.resize(0);
		path = FindPath( _start, _goal );
		diversePaths.push_back( path );
		nodePaths.push_back( mPath );
		// Update the values
		allNodePaths = JoinPaths( nodePaths );
		UpdateNodeValues( allNodePaths );
		// Reset the search
		ResetSearch();
	}
	
	return diversePaths;
}

/**
 * @function JoinPaths
 * @brief FIXME CUT OFF REPEATED GUYS	
 */
std::vector<int> Search::JoinPaths( std::vector< std::vector<int> >  _allPaths ) {

	std::vector<int> bunchPaths;

	for( int i = 0; i < _allPaths.size(); ++i ) {
		for( int j = 0; j < _allPaths[i].size(); ++j ) {
			bunchPaths.push_back( _allPaths[i][j] );			
		}
	}

	return bunchPaths;
}

/**
 * @functio ResetSearch 
 */
void Search::ResetSearch() {

	for( int i = 0; i < mNumNodes; ++i ) {
		Cost c; c.F = SEARCH_INF; c.F = SEARCH_INF; c.H = SEARCH_INF;
		mNodes[i].cost = c; 
		mNodes[i].status = IN_NO_SET;
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
	for( int i = 0; i < mNumNodes; ++i ) {
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
	double minBrushDist = SEARCH_INF;
	double maxBrushDist = -SEARCH_INF;

	for( int i = 0; i < mNumNodes; ++i ) {
		if( mNodes[i].brushDistance < minBrushDist ) {
			minBrushDist = mNodes[i].brushDistance;
		}

		if( mNodes[i].brushDistance > maxBrushDist ) {
			maxBrushDist = mNodes[i].brushDistance;
		}
		
	}

	printf("Min and max: %f , %f \n", minBrushDist, maxBrushDist );


	//-- Update accordingly
	for( int i = 0; i < mNumNodes; ++i ) {
		mNodes[i].value = ( maxBrushDist - mNodes[i].brushDistance ) + sNominalValue;
	}

}

/**
 * @function FindPath
 */
std::vector<Pos> Search::FindPath( Pos _start, Pos _goal ) {
	
	int startNode; int goalNode;

	//-- Search
	Eigen::VectorXd eStart(3);
	eStart << (double)_start.x, (double)_start.y, (double)_start.z; 
    struct kdres* result = kd_nearest( mKdTree, eStart.data() );
	startNode = (int) kd_res_item_data(result);

	Eigen::VectorXd eGoal(3);
	eGoal << (double)_goal.x, (double)_goal.y, (double)_goal.z; 
    result = kd_nearest( mKdTree, eGoal.data() );
	goalNode = (int) kd_res_item_data(result);

	kd_res_free( result );


	printf("Start: %d Goal: %d \n", startNode, goalNode );
	printf( "-- Start search A* \n" );
	
	//-- Fill the starting state
	Cost cost;
	cost.G = 0;
	cost.H = costHeuristic( startNode, goalNode );
    cost.F = cost.G + mEpsilon*cost.H;
	mNodes[startNode].cost = cost;
	mNodes[startNode].parent = -1;
	mNodes[startNode].status = IN_NO_SET;
		
	//-- Push it into the open set
	pushOpenSet( startNode );
	
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
		if( x == goalNode ) {
		
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
			double tentative_G_score = mNodes[x].cost.G + edgeCost( x, y, mNodes[y].value );
			
			if( mNodes[y].status != IN_OPEN_SET ) {
				mNodes[y].parent = x;
				cost.G = tentative_G_score;
				cost.H = costHeuristic( y, goalNode );
				cost.F = cost.G + mEpsilon*cost.H;
				mNodes[y].cost = cost;
				pushOpenSet(y);
			}
			else {
				if( tentative_G_score < mNodes[y].cost.G ) {
					mNodes[y].parent = x;
					cost.G = tentative_G_score;
					cost.H = costHeuristic( y, goalNode );
					cost.F = cost.G + mEpsilon*cost.H;
					mNodes[y].cost = cost;
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
	Pos pStart = mNodes[_start].pos;
	Pos pGoal = mNodes[_goal].pos;

	double dx = pStart.x - pGoal.x;
	double dy = pStart.y - pGoal.y;
	double dz = pStart.z - pGoal.z;

	return sqrt( dx*dx + dy*dy + dz*dz )*sNominalValue;

}

/** 
 * @function edgeCost
 */
double Search::edgeCost( int _nodeFrom, int _nodeTo, double _value ) {

	Pos pFrom = mNodes[_nodeFrom].pos;
	Pos pTo = mNodes[_nodeTo].pos;

	double sum = abs(pFrom.x - pTo.x) + abs(pFrom.y - pTo.y) + abs(pFrom.z - pTo.z);
	if( sum == 1.0 ) {
		return 1.0*_value;
	}	
	else if( sum == 2.0 ) {
		return sqrt(2)*_value;
	}
	else if( sum == 3.0 ) {
		return sqrt(3)*_value;
	}

	else {
		printf( " [edgeCost] What the heck? ERROR in getting neighbors! \n" ); 
		return SEARCH_INF; 
	}
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
    { HT[_key] = n;
	  return; }

  // If not, start on the bottom and go up
  node = n;

  int qp; int qn;

  while( node != 0 )
  {
    parent = floor( (node - 1)/2 );
	qp = mOpenSet[parent];
    qn = mOpenSet[node];
    // Always try to put new nodes up
    if( mNodes[qp].cost.F >= mNodes[qn].cost.F )
      {
        temp = mOpenSet[parent];
        mOpenSet[parent] = qn; HT[qn] = parent; 
        mOpenSet[node] = temp; HT[temp] = node; 
        node = parent; 
      }  
    else
     { break; }
  }   

  HT[_key] = node;

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

  mOpenSet[0] = mOpenSet[bottom]; HT[ mOpenSet[bottom] ] = 0;
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
       if( mNodes[ mOpenSet[node] ].cost.F >= mNodes[ mOpenSet[child_1] ].cost.F )
        { u = child_1;  }
       if( mNodes[ mOpenSet[u] ].cost.F  >= mNodes[ mOpenSet[child_2] ].cost.F )
        { u = child_2; }
     }
    else if( child_1 < n )
     {
       if( mNodes[ mOpenSet[node] ].cost.F >= mNodes[ mOpenSet[child_1] ].cost.F )
         { u = child_1; }
     }
    
	qu = mOpenSet[u];
    if( node != u )
     { temp = mOpenSet[node];
       mOpenSet[node] = qu; HT[qu] = node;
       mOpenSet[u] = temp; HT[temp] = u; 
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
  n = HT[key];
/*
  for( int i = 0; i < mOpenSet.size(); i++ )
  {
    if( mOpenSet[i] == key )
    { n = i; break; }
  }
*/

  //-- If it happens to be the first element
  if( n == 0 )
    { return; } //-- HT[ID] = 0; // the same

  //-- If not, start on the bottom and go up
  node = n;

  int qp; int qn;

  while( node != 0 )
  { 
    parent = floor( (node - 1)/2 );

	qp = mOpenSet[parent]; qn = mOpenSet[node];
    // Always try to put new nodes up
    if( mNodes[ qp ].cost.F > mNodes[ qn ].cost.F )
      {
        //printf(" Parent pos: %d f: %f \n ", parent, nodes_[ openSet_[parent] ].f_score );
        temp = mOpenSet[parent];
        mOpenSet[parent] = qn; HT[qn] = parent;
        mOpenSet[node] = temp; HT[temp] = node; 
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
  printf("--> Trace path \n");
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
	   p = mNodes[mPath[i] ].pos;
	   _posPath.push_back( p ); 
	 }

  if( _posPath.size() > 0 )
    { return true; }

  return false;  
}  


