/**
 * @file HomoSearch.h
 * @author A. Huaman
 */

#ifndef _HOMO_SEARCH_H_
#define _HOMO_SEARCH_H_

#include "distance_field/include/distance_field/collision_map.h"
#include "kdtree/kdtree.h"


using namespace distance_field;

struct Node {
	int index;
	std::vector<int> neighbors;

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
 * @class HomoSearch
 */
class HomoSearch {

public:
	HomoSearch( CollisionMap *_collisionMap,
				std::vector<Eigen::Vector3i> *_ridge );
	~HomoSearch();
	
	void searchRidge();

 	//-- Search specific
	std::vector<Eigen::Vector3i> getPath( int _start, int _goal );
	void pushOpenSet( int _key );
	int popOpenSet();
	void updateLowerOpenSet( int key );
	bool tracePath( const int &key, std::vector< Eigen::Vector3i> & path );
	double costHeuristic( int _start, int _goal );
	double edgeCost( int _nodeA, int _nodeB );

	//-- Instance variables
	std::vector<Eigen::Vector3i> *mRidge;
	CollisionMap *mCollisionMap;
	std::vector<Node> mNodes;

	int mNumCells;
	double mCellNeighborRange;
	struct kdtree *mKdTree;

	//-- Class constant
	static const int sMaxIter;

	//-- Search specific stuff
	std::vector< int > mOpenSet;
    std::vector< Eigen::Vector3i> mPath;
	double mEpsilon;

};


#endif /** _HOMO_SEARCH_H_ */
