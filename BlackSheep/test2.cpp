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

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "BlackSheep.h"

/**
 * @
 */
int main( int argc, char* argv[] ) {
	srand( time(NULL) );
	BS bs( 80, 80, 80 );

	bs.CreateExternalBoundary();
	bs.CreateBox(  20, 10, 20, 40, 60, 40 );


	//-- Debugging. Damn
	time_t ts; time_t tf; double dt; 
    ts = clock();
    bs.CalculateDT();
	tf = clock();
	printf("--** End DT construction process: Time elapsed: %.3f  \n", (double) (tf - ts)/CLOCKS_PER_SEC );


	std::vector< std::vector<Eigen::Vector3i> > paths;
	int startX = 30;
	int startY = 40;
	int startZ = 15;
	int goalX = 40;
	int goalY = 40; 
	int goalZ = 70;
	float epsilon = 1.0;
	int numPaths = 4;

	paths = bs.FindVarietyPaths( startX, startY, startZ, goalX, goalY, goalZ, numPaths, epsilon );

    pcl::visualization::PCLVisualizer *viewer;
	viewer = new pcl::visualization::PCLVisualizer( "Test Black Sheep" );

	//bs.ViewVoronoi( viewer, 255, 0, 0 );
	bs.ViewObstacles( viewer, 0, 0, 255 );
	for( size_t i = 0; i < paths.size(); ++i ) {
		int r = rand() % 256;
		int g = rand() % 256;
		int b = rand() % 256;
		bs.ViewPath( paths[i], viewer, r, g, b);
	}
	while( !viewer->wasStopped() ) {
		viewer->spin();
	}

	return 0;
}


