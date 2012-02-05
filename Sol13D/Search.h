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

/**
 * @file GridSearch.h
 * @author A. Huaman Quispe <ahuaman3@gatech.edu>
 */

#ifndef _GRID_SEARCH_H_
#define _GRID_SEARCH_H_

#include "Grid3D.h"
#include "TopologyGrid3D.h"
#include "kdtree/kdtree.h"
#include <Eigen/Core>
#include <vector>
#include <stdlib.h>

/**
 * @struct Cost
 */
struct Cost {
	double F;
	double G;
	double H;
};

/**
 * @struct Node
 */
struct Node {
	int index;
	Pos pos;
	std::vector<int> neighbors;
	double value;
	double brushDistance; // For updating the nodes' values
	Cost cost;
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
	std::vector<Pos> FindPath( Pos _start, Pos _goal );
	std::vector< std::vector<Pos> > FindDiversePaths( Pos _start, Pos _goal, int _times );
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
	Node *mNodes;

	int mNumNodes;
	double mNodeNeighborRange;
	struct kdtree *mKdTree;

	//-- Class constant
	static const int sMaxIter;
	static const double sNominalValue;
	static const double SEARCH_INF;

	//-- Search specific stuff
	std::vector< int > mOpenSet;
	int *HT;
	double mEpsilon;
	std::vector<int> mPath;

};


#endif /** _GRID_SEARCH_H_ */
