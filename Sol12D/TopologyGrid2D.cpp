/**
 * @file TopologyGrid2D.cpp
 */

#include "TopologyGrid2D.h"
#include <stdio.h>
#include <cstdlib>

int TopologyGrid2D::NX[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
int TopologyGrid2D::NY[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
const int TopologyGrid2D::TG2D_INF = 1000;

/**
 * @function Constructor
 */
TopologyGrid2D::TopologyGrid2D( Grid2D *_g ) {

	mG = _g;
	mNumCells = mSizeX*mSizeY;	
	mSizeX = _g->GetSizeX();	
	mSizeY = _g->GetSizeY();

    Cell cell; Pos p; int index;

	//-- Create basic nodes
	index = 0;
	for( int j = 0; j <	_g->GetSizeY(); ++j ) {
		for( int i = 0; i < _g->GetSizeX(); ++i ) {

			p.x = i; p.y = j;
			cell.pos = p;
			cell.distance = TopologyGrid2D::TG2D_INF; // No yet check obstacles

			cell.index = index;
			cell.neighbors = GetNeighbors( p );
			mCells.push_back( cell );
			
			index++;			
		}		
	}
	
}

/**
 * @function Destructor
 */
TopologyGrid2D::~TopologyGrid2D( ) {

}

/**
 * @function CalculateDT
 */
void TopologyGrid2D::CalculateDT() {

	std::vector<int> queue;
	
	//-- 1. Initialize queue with obstacle grids and set distances to zero for them
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {

			Pos p; p.x = i; p.y = j;
			int ind = ref( i, j );

			if( mG->GetState( p ) == true ) {
				queue.push_back( ind );
				mCells[ ref( p.x, p.y )].distance = 0;
			}
		}
	}

	//-- 2. Loop until no more new elements in Queue
	std::vector<int> newQueue(0);

	while( queue.size() > 0 ) {

		for( int i = 0; i < queue.size(); ++i ) {
			std::vector<int> n = mCells[ queue[i] ].neighbors;
			int dist = mCells[ queue[i] ].distance;

			for( int j = 0; j < n.size(); ++j ) {
				int new_dist = dist + EdgeCost( queue[i], n[j] );
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
int TopologyGrid2D::EdgeCost( int n1, int n2 ) {

	int dx = abs( mCells[n1].pos.x - mCells[n2].pos.x ); 
	int dy = abs( mCells[n1].pos.y - mCells[n2].pos.y ); 
    int d = dx + dy;

	if( d == 2 ) {
		return 14; // sqrt(2)
	}
	if( d == 1 ) {
		return 10; // 1
	}
	else {
		printf( "--> WTH. There is an error here! \n" );
	}
}

/**
 * @function GetDTRidge
 */
std::vector<Cell> TopologyGrid2D::GetDTRidge() {

	std::vector<Cell> ridge;
	Pos p;
	
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {

			int x = ref( i, j );
			p.x = i, p.y = j; 
			if( mG->GetState( p ) != true && IsLocalMaxima( x ) == true ) {		
				ridge.push_back( mCells[x] );
			}			
		}
	}	

	return ridge;
}

/**
 * @function GetFreeCells
 */
std::vector<Cell> TopologyGrid2D::GetFreeCells() {

	std::vector<Cell> free;
	Pos p;

	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {

			int x = ref( i, j );
			p.x = i, p.y = j; 
			if( mG->GetState( p ) != true) {		
				free.push_back( mCells[x] );
			}			
		}
	}	

	return free;
	
}


/**
 * @function IsLocalMaximum
 */
bool TopologyGrid2D::IsLocalMaxima( int _index ) {

	Cell c = mCells[ _index ];
	int dist = c.distance;		
	std::vector<int> nx = c.neighbors;
	for( int i = 0; i < nx.size(); ++i ) {
		Cell n = mCells[nx[i]];
		if( EdgeCost( nx[i], _index ) == 10 && n.distance >= c.distance + 10  ) {
			return false;		
		}
		else if( EdgeCost( nx[i], _index ) == 14 && n.distance >= c.distance  + 14 ) {		
			return false;
		}
	}

	return true;
}


/**
 * @function GetNeighbors	
 */
std::vector<int> TopologyGrid2D::GetNeighbors( Pos _p ) {

	std::vector<int> neighbors;	
	int x = _p.x;
	int y = _p.y;
	int nx; int ny;

	for( int i = 0; i < 8; ++i ) {
		nx = x + NX[i];
		ny = y + NY[i];
		if( IsValid( nx, ny ) == true ) {
			neighbors.push_back( ref(nx, ny) );
		}
	}		
	return neighbors;		
}

/**
 * @function IsValid
 */
bool TopologyGrid2D::IsValid( int _x, int _y ) {

	if( _x < 0 || _x >= mSizeX || _y < 0 || _y >= mSizeY ) {
		return false;
	}

	return true;
}

/**
 * @function ref
 */
int TopologyGrid2D::ref( int _x, int _y) {
	return _y*mSizeX + _x;
}

/**
 * @function PrintDT	
 */
void TopologyGrid2D::PrintDT() {
	for( int i = 0; i < mSizeX; ++i ) {
		for( int j = 0; j < mSizeY; ++j ) {
			printf( " %d ", mCells[ ref(i,j) ].distance );
		}
		printf("\n");
	}
}
