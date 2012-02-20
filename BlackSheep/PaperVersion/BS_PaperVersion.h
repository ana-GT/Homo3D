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

#ifndef _BLACK_SHEEP_PAPER_VERSION_
#define _BLACK_SHEEP_PAPER_VERSION_

//-- PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <vector>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <time.h>

/** State Enum */
enum {
	FREE_STATE = 1,
	OBSTACLE_STATE = 2
};

/** Status Enum */
enum {
	IN_CLOSED_SET = 2,
	IN_OPEN_SET = 3,
	IN_NO_SET = 4
};

/**
 * @struct SearchData
 */
struct SearchData {
	float costF;
	float costG;
	float costH;
	
	float value;
	float brushDist;	// For updating nodes' values
	int status;
	int parent;
	
};

/**
 * @struct Node
 */
struct Node3D {
	int x; 
	int y; 
	int z;
	float obsDist;
	int index;
	int state;

	SearchData s;
};


/**
 * @class BlackSheep
 */
class BS {

	public:
		//-- Constructor
		BS( int _sizeX, int _sizeY, int _sizeZ );
		//-- Destructor
		~BS();

		//-- Grid functions
		void SetState( int _x, int _y, int _z, int _state );
		int ref( int _x, int _y, int _z ) const;
		int GetState( int _x, int _y, int _z ) const;
		int GetState( int _ref ) const;
		int GetSizeX() const;
		int GetSizeY() const;
		int GetSizeZ() const;
		bool CheckCollision( int _x, int _y, int _z );
		
		//-- DT Functions
		bool IsValid( int _x, int _y, int _z ) const;
		float EdgeCost( int _index1, int _index2, float _value = sNominalValue );
		std::vector<int> GetNeighbors( int _x, int _y, int _z ) const;
		std::vector<int> GetNeighbors( int _ref ) const;
		void CalculateDT();
		std::vector<Node3D*> GetDTRidge();
		bool IsLocalMaxima( int _index );

		//-- Visualization Functions
		void ViewObstacles( pcl::visualization::PCLVisualizer *_viewer,
							int _r = 255, int _g = 0, int _b = 0 );
		void ViewVoronoi( pcl::visualization::PCLVisualizer *_viewer,
					      int _r = 255, int _g = 0, int _b = 0 );
		void ViewPath( std::vector<Eigen::Vector3i> _path, pcl::visualization::PCLVisualizer *_viewer,
					   int _r = 255, int _g = 255, int _b = 0 );
		void ViewPaths( std::vector< std::vector<Eigen::Vector3i> > _paths, pcl::visualization::PCLVisualizer *_viewer );

		//-- Geometry Helper  functions
		void CreateExternalBoundary();
		void CreateBox( int _x, int _y, int _z, int _dimX, int _dimY, int _dimZ );

		//-- Search especific functions
		std::vector<Eigen::Vector3i> FindPath( int _x1, int _y1, int _z1,
											   int _x2, int _y2, int _z2 );
		
		std::vector< std::vector<Eigen::Vector3i> > FindVarietyPaths( int _x1, int _y1, int _z1,
										   				  		  int _x2, int _y2, int _z2, int _times, float _alpha = 1.0 );

		void CalculateDTPaths( std::vector<int> _path ); 
	    void UpdateNodeValues( std::vector<int> _path );
		std::vector<int> JoinPaths( std::vector< std::vector<int> >  _allPaths );
    	void ResetSearch();

		void InitSearch( );
		void PushOpenSet( int _key );
		int PopOpenSet();
		void UpdateLowerOpenSet( int key );
		bool TracePath( const int &key, std::vector<Eigen::Vector3i> & _path );
		float CostHeuristic( int _start, int _target  );

		//-- Stuff!
		static const int NX[];
		static const int NY[];
		static const int NZ[];

		static const float BS_INF;
		static const float BS_SQRT2;
		static const float BS_SQRT3;	
		static const float sNominalValue;
		static const float sDefaultBrushDist;
		static const int sMaxIter; 


	private:
		int mSizeX;
		int mSizeY;
		int mSizeZ;
		int mNumNodes;

		int mStride1;
		int mStride2;

		Node3D *mNodes;
		std::vector<int> *mGeometricNeighbors;

		// Auxiliar stuff for search
		int *mHT;
		std::vector< int > mOpenSet;
		std::vector<int> mPath;
		int mCountPaths;
		float mAlpha;
		float mMaxBrushDist;
	
};

/////////// INLINE FUNCTIONS ////////////////////////

inline int BS::GetSizeX() const {
	return mSizeX;
}

inline int BS::GetSizeY() const {
	return mSizeY;
}

inline int BS::GetSizeZ() const {
	return mSizeZ;
}

inline int BS::ref( int _x, int _y, int _z ) const {
	return  _x*mStride1 + _y*mStride2 + _z;
}


#endif /** _BLACK_SHEEP_PAPER_VERSION_ */
