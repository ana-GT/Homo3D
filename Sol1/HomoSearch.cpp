/**
 * @file HomoSearch.cpp
 */

#include "HomoSearch.h"

const int HomoSearch::sMaxIter = 50000;

/**
 * @function HomoSearch
 */
HomoSearch::HomoSearch( CollisionMap *_collisionMap,
						std::vector<Eigen::Vector3i> *_ridge ) {

	mCellNeighborRange = 1.8; // bigger than sqrt(3), smaller than 2
	mEpsilon =1.5;

	mCollisionMap = _collisionMap;
	mRidge = _ridge;
	mNumCells = mRidge->size(); 
}

/**
 * @function HomoSearch
 */
HomoSearch::~HomoSearch() {
	kd_free( mKdTree );
}


/**
 * @function searchRidgeSpace
 */
void HomoSearch::searchRidge() {

	
	//-- 2. Add points to kd-tree
	mKdTree = kd_create( 3 ); // 3 dimensional space

	for( int i = 0; i < mNumCells; ++i ) { 
		uintptr_t id = i; 
		Eigen::VectorXd entity = (*mRidge)[i].cast<double>();
		kd_insert( mKdTree, entity.data(), (void*)id );
	}
	
	//-- Fill the nodes
	for( int i = 0; i < mNumCells; ++i ) {
		Eigen::VectorXd entity = (*mRidge)[i].cast<double>();

	    Node node; node.index = i; node.status = IN_NO_SET;

		struct kdres* neighset = kd_nearest_range( mKdTree, entity.data(), mCellNeighborRange );

		int size = kd_res_size( neighset );
		if( size == 0 || size >= 26 ) {
			printf("Oh boy, problem here \n");
		}


		for( int j = 0; j < size; ++j ) {
		    uintptr_t near = (uintptr_t) kd_res_item_data( neighset ); 

			if( near != node.index ) { 
				node.neighbors.push_back( near );
			}
			kd_res_next( neighset );			
		}

	    mNodes.push_back( node );

		//-- Free the structure
		kd_res_free( neighset );
	}

	//-- Search
	int start = 0;
	int goal = mNumCells - 1;
	std::vector<Eigen::Vector3i> path = getPath( start, goal );	
		
}

/**
 * @function costHeuristic
 */
double HomoSearch::costHeuristic( int _start, int _goal ) {
	return ( ( (*mRidge)[ mNodes[_start].index ] - (*mRidge)[ mNodes[_goal].index ] ).cast<double>() ).norm();
}

/** 
 * @function edgeCost
 */
double HomoSearch::edgeCost( int _nodeA, int _nodeB ) {
	Eigen::Vector3i diff = (*mRidge)[_nodeA] - (*mRidge)[_nodeB];
	int sum = abs( diff(0) ) + abs( diff(1) ) + abs( diff(2) );
	if( sum == 1 ) {
		return 1.0;
	}	
	if( sum == 2 ) {
		return 1.41;
	}
	if( sum == 3 ) {
		return 1.71;
	}
	else {printf("[edgeCost] What the heck? ERROR in getting neighbors!!!!!! \n"); return 1000;}
}


/**
 * @function getPath
 */
std::vector<Eigen::Vector3i> HomoSearch::getPath( int _start, int _goal ) {

	printf( "-- Start search A* \n" );
	
	//-- Fill the starting state
	mNodes[_start].costG = 0;
	mNodes[_start].costH = costHeuristic( _start, _goal );
	mNodes[_start].costF = mNodes[_start].costG + mEpsilon*mNodes[_start].costH;
	mNodes[_start].parent = -1;
	mNodes[_start].status = IN_NO_SET;
		
	//-- Push it into the open set
	pushOpenSet( _start );
	
	//-- Loop
	int x;
	int count = 0;

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
		if( x == _goal ) {
		
			printf( "--> Found a path! \n" ); tracePath( x, mPath ); break;
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
			double tentative_G_score = mNodes[x].costG + edgeCost( x, y );
			
			if( mNodes[y].status != IN_OPEN_SET ) {
				mNodes[y].parent = x;
				mNodes[y].costG = tentative_G_score;
				mNodes[y].costH = costHeuristic( y, _goal );
				mNodes[y].costF = mNodes[y].costG + mEpsilon*mNodes[y].costH;
				pushOpenSet(y);
			}
			else {
				if( tentative_G_score < mNodes[y].costG ) {
					mNodes[y].parent = x;
					mNodes[y].costG = tentative_G_score;
					mNodes[y].costH = costHeuristic( y, _goal );
					mNodes[y].costF = mNodes[y].costG + mEpsilon*mNodes[y].costH;
					//-- Reorder your OpenSet
					updateLowerOpenSet( y );				
				}
			}
		} //-- End for every neighbor

		
	} //-- End of while

	printf("I am done \n");
	return mPath;
}

/**
 * @function pushOpenSet
 * @brief push the node ID into the openSet binary heap
 */
void HomoSearch::pushOpenSet( int _key ) { 

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
int HomoSearch::popOpenSet() { 

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
void HomoSearch::updateLowerOpenSet( int key ) { 
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
bool HomoSearch::tracePath( const int &key, std::vector< Eigen::Vector3i> & path )
{
  printf("Trace path \n");
  std::vector< Eigen::Vector3i > backPath;
  path.resize(0);
  backPath.resize(0);

  int n = key;
  Eigen::VectorXi wp; wp.resize(3);

  while( n != -1 ) 
  {
    backPath.push_back( (*mRidge)[n] );
    n = mNodes[n].parent;
  }

  int b = backPath.size();

  for( int i = 0; i < b; i++ )
     { path.push_back( backPath[ b - 1- i] ); }

  if( path.size() > 0 )
    { return true; }

  return false;  
}  

