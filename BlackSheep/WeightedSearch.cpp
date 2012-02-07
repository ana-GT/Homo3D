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

#include "BlackSheep.h"


/**
 * @function FindVarietyWeightedPaths
 */
std::vector< std::vector<Eigen::Vector3i> > BS::FindVarietyWeightedPaths( int _x1, int _y1, int _z1,
									   			   	  		      		  int _x2, int _y2, int _z2, 
																		  int _times, float _epsilon, 
																		  float _alpha ) {

	mEpsilon = _epsilon;
	mAlpha = _alpha;

	std::vector< std::vector<Eigen::Vector3i> > paths;
    std::vector<Eigen::Vector3i> path;
	std::vector< std::vector<int> > nodePaths;
	std::vector<int> allNodePaths;

	InitSearch();

	for( size_t i = 0; i < _times; ++i ) {
		path.resize(0);
		path = FindWeightedPath( _x1, _y1, _z1, _x2, _y2, _z2 );
		paths.push_back( path );
		nodePaths.push_back( mPath );
		//-- Update the values
		allNodePaths = JoinPaths( nodePaths );
		UpdateNodeValues( allNodePaths );
		// Reset the search
		ResetSearch();
	}

	return paths;
}


/**
 * @function FindWeightedPath
 */
std::vector<Eigen::Vector3i> BS::FindWeightedPath( int _x1, int _y1, int _z1,
										   		   int _x2, int _y2, int _z2 ) {

	std::vector<Eigen::Vector3i> path;
	printf( "Start Search Weighted A* \n" );

	if( IsValid(_x1, _y1, _z1) == false || IsValid(_x2, _y2, _z2) == false ) {
		printf("Error, Start or Target NO valid, Exiting \n" ); return path;
	}

	int nodeStart = ref( _x1, _y1, _z1 );
	int nodeTarget = ref( _x2, _y2, _z2 );


	//-- Fill start space
	Node3D *node;
	node = &mNodes[nodeStart];
	node->s.costG = 0;
	node->s.costH = CostHeuristic( nodeStart, nodeTarget ) + mAlpha*CostHeuristic( nodeStart, nodeTarget )*sNominalValue/mMaxBrushDist;
	node->s.costF = node->s.costG + mEpsilon*node->s.costH;
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
			double tentative_G_score = mNodes[x].s.costG + mEpsilon*( EdgeCost( x, y ) + mAlpha*EdgeCost( x, y )*mNodes[y].s.value/mMaxBrushDist );
			
			if( mNodes[y].s.status != IN_OPEN_SET ) {
				node = &mNodes[y];
				node->s.parent = x;
				node->s.costG = tentative_G_score;
				// Length cost + alpha*node_value_cost
				node->s.costH = CostHeuristic( y, nodeTarget ) + mAlpha*CostHeuristic( y, nodeTarget )*sNominalValue/mMaxBrushDist;
				node->s.costF = node->s.costG + mEpsilon*node->s.costH;
				PushOpenSet(y);
			}
			else {
				if( tentative_G_score < mNodes[y].s.costG ) {
					node = &mNodes[y];
					node->s.parent = x;
					node->s.costG = tentative_G_score;
					node->s.costH = CostHeuristic( y, nodeTarget ) + mAlpha*CostHeuristic( y, nodeTarget )*sNominalValue/mMaxBrushDist;
					node->s.costF = node->s.costG + mEpsilon*node->s.costH;
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

