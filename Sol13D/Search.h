/**
 * @file GridSearch.h
 * @author A. Huaman
 */

#ifndef _GRID_SEARCH_H_
#define _GRID_SEARCH_H_

#include "Grid3D.h"
#include "TopologyGrid3D.h"
#include "kdtree/kdtree.h"
#include <Eigen/Core>
#include <vector>
#include <stdlib.h>


struct Node {
	int index;
	double x; double y; double z;
	std::vector<int> neighbors;
	double value;
	double brushDistance; // For updating the nodes' values

	double costF;
	double costG;
	double costH;
	int status;
	int parent;
};

enum {
	IN_CLOSED_SET = 2,
	IN_OPEN_SET = 3,
	IN_NO_SET = 4
};

/**
 * @class Search
 */
class Search {

public:
	Search( std::vector<Cell> *_cells );
	~Search();
	

 	//-- Search specific
	std::vector<Pos> FindPath( double _startX, double _startY, double _startZ, double _goalX, double _goalY, double _goalZ );
	std::vector< std::vector<Pos> > FindDiversePaths( double _startX, double _startY, double _startZ, double _goalX, double _goalY, double _goalZ, int _times );
    void UpdateNodeValues( std::vector<int> _path );
	std::vector<int> JoinPaths( std::vector< std::vector<int> >  _allPaths );
    void ResetSearch();
	void pushOpenSet( int _key );
	int popOpenSet();
	void updateLowerOpenSet( int key );
	bool tracePath( const int &key, std::vector<Pos> & _PosPath );
	double costHeuristic( int _start, int _goal );
	double edgeCost( int _nodeFrom, int _nodeTo, double _value );

	//-- Instance variables
	std::vector<Node> mNodes;

	int mNumCells;
	double mCellNeighborRange;
	struct kdtree *mKdTree;

	//-- Class constant
	static const int sMaxIter;
	static const int sNominalValue;
	static const double SEARCH_INF;

	//-- Search specific stuff
	std::vector< int > mOpenSet;
	double mEpsilon;
	std::vector<int> mPath;

};


#endif /** _GRID_SEARCH_H_ */
