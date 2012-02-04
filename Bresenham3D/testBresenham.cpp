/**
 * @file testBresenham.cpp
 */


#include "Bresenham3D.h"
#include <stdio.h>

void Draw3DLine( std::vector<Eigen::Vector3i> _line );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

	std::vector<Eigen::Vector3i> line;
	BresenhamLine3D( 2,3,4, 20, 30, 25, line );
	Draw3DLine( line );
	return 0;
}

void Draw3DLine( std::vector<Eigen::Vector3i> _line ) {

	int r;
	FILE *g = popen( "gnuplot -persist", "w" );

	assert(g);
	
	fprintf( g, "splot '-' with lines ");
	fprintf( g, "\n" );
	for( size_t i = 0; i < _line.size(); ++i ) {
		fprintf( g, "%d %d %d\n", _line[i](0), _line[i](1), _line[i](2) );
	}
	fprintf( g, "e\n" );

	fflush(g);
	r = pclose(g);
}
