/**
 * @file TopologyGrid2D
 * @brief WTH
 */

#ifndef _TOPOLOGY_GRID_2D_
#define _TOPOLOGY_GRID_2D_

#include <vector>
#include "Grid2D.h"


/**
 * @struct Cell
 */
struct Cell {
	Pos pos;
	std::vector<int> neighbors;
	int distance;
	int index;
};


/**
 * @class Topology
 */
class TopologyGrid2D {

public:
	TopologyGrid2D( Grid2D *_g );
	~TopologyGrid2D();	
	int ref( int _x, int _y);
	bool IsValid( int _x, int _y );
	int EdgeCost( int n1, int n2 );
	std::vector<int> GetNeighbors( Pos _p ); 
    void CalculateDT();
	std::vector<Cell> GetDTRidge();
	std::vector<Cell> GetFreeCells(); 
	bool IsLocalMaxima( int _index );
	void PrintDT();

	//-- Instance variables
	Grid2D *mG;
	int mNumCells;
	int mSizeX;
	int mSizeY;
	std::vector<Cell> mCells;

	static int NX[];
	static int NY[];
	static const int TG2D_INF;
};



#endif /** _TOPOLOGY_GRID_2D_ */
