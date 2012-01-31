/**
 * @file TopologyGrid3D
 * @brief WTH
 */

#ifndef _TOPOLOGY_GRID_3D_
#define _TOPOLOGY_GRID_3D_

#include <vector>
#include "Grid3D.h"


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

class TopologyGrid3D {

public:
	TopologyGrid3D( Grid3D *_g );
	~TopologyGrid3D();	
	int ref( int _x, int _y, int _z );
	bool IsValid( int _x, int _y, int _z );
	int EdgeCost( int n1, int n2 );
	std::vector<int> GetNeighbors( Pos _p ); 
    void CalculateDT();
	std::vector<Cell> GetDTRidge();
	std::vector<Cell> GetFreeCells(); 
	bool IsLocalMaxima( int _index );

	void VisualizeRidge();

	//-- Instance variables
	Grid3D *mG;
	int mNumCells;
	int mSizeX;
	int mSizeY;
	int mSizeZ;
	int mStride1;
	int mStride2;
	std::vector<Cell> mCells;

	static int NX[];
	static int NY[];
	static int NZ[];
	static const int TG3D_INF;
};



#endif /** _TOPOLOGY_GRID_3D_ */
