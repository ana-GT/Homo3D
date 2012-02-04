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

#include "Bresenham3D.h"

/**
 * @function BresenhamLine3D
 * @brief Spits out the voxels belonging to the line
 * @date 2012-02-04	
 */
void BresenhamLine3D( int _x1, int _y1, int _z1,
					  int _x2, int _y2, int _z2,
					  std::vector<Eigen::Vector3i> &_line ) {

	int x1, y1, z1;
	int x2, y2, z2;
	int dx; int dy; int dz;
	int d;
	int sx; int sy; int sz;
	int ax; int ay; int az;
	int xd; int yd; int zd;
	int x; int y; int z;
	int *X; int *Y; int *Z;
	int idx;

	x1 = _x1; y1 = _y1; z1 = _z1;
	x2 = _x2; y2 = _y2; z2 = _z2;

	dx = ( x2 - x1 );
	dy = ( y2 - y1 );
	dz = ( z2 - z1 );

	if( dx >= dy && dx >= dz ) { d = dx + 1; }
	else if( dy >= dx && dy >= dz ) { d = dy + 1; }
	else if( dz >= dx && dz >= dy ) { d = dz + 1; }

	X = new int[d]; 
	Y = new int[d];
	Z = new int[d];

	ax = abs(dx)*2;
	ay = abs(dy)*2;
	az = abs(dz)*2;
		
	if( dx < 0 ) { sx = -1; } 
    else { sx = 1; }

	if( dy < 0 ) { sy = -1; } 
    else { sy = 1; } 

	if( dz < 0 ) { sz = -1; } 
	else { sz = 1; }

	//-- Set the initial point
	x = x1; y = y1; z = z1; idx = 1;

	//-- x difference is the biggest
	if( ax >= ay && ax >= az ) {
		yd = ay - ax / 2;
		zd = az - ax / 2;

		while( true ) {
			X[idx] = x; Y[idx] = y; Z[idx] = z;
			idx = idx + 1;

			if( x == x2 ) { break; }
			if( yd >= 0 ) {
				y = y + sy;
				yd = yd - ax;
			}
			if ( zd >= 0 ) {
				z = z + sz;
				zd = zd - ax;
			}			

			x = x + sx;
			yd = yd + ay;
			zd = zd + az;
		}
	}

	//-- y difference is the biggest
	else if( ay >= ax && ay >= az ) {
		xd = ax - ay / 2;
		zd = az - ay / 2;

		while( true ) {
			X[idx] = x; Y[idx] = y; Z[idx] = z;
			idx = idx + 1;

			if( y == y2 ) { break; }
			if( xd >= 0 ) {
				x = x + sx;
				xd = xd - ay;
			}
			if ( zd >= 0 ) {
				z = z + sz;
				zd = zd - ay;
			}			

			y = y + sy;
			xd = xd + ax;
			zd = zd + az;
		}
	}

	//-- z difference is the biggest
	else if( az >= ax && az >= ay ) {
		xd = ax - az / 2;
		yd = ay - az / 2;

		while( true ) {
			X[idx] = x; Y[idx] = y; Z[idx] = z;
			idx = idx + 1;

			if( z == z2 ) { break; }
			if( xd >= 0 ) {
				x = x + sx;
				xd = xd - az;
			}
			if ( yd >= 0 ) {
				y = y + sy;
				yd = yd - az;
			}			

			z = z + sz;
			xd = xd + ax;
			yd = yd + ay;
		}
	}

	//-- Return it in a vector
	_line.resize(0);
	for( int i = 0; i < d; ++i ) {
		Eigen::Vector3i p;
		p << X[i], Y[i], Z[i];
		_line.push_back(p);
	}
	
}

