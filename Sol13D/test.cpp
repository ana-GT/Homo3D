/**
 * @file test.cpp
 */

//-- PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


#include "test.h"

//////////// Extern Variables /////////////////////

Pos mStart;
Pos mGoal;
Search *mSearch;;
std::vector< std::vector<Pos> > gPaths;
std::vector<Cell> gRidge;
int numPaths;
int countPath;
/////////////////////////////////////////////////

// Functions
void DrawResult( Grid3D *_g, std::vector< std::vector<Pos> >  _paths );

/**
 * @function main  
 */
int main( int argc, char *argv[] ) {

	//-- Create grid
    Grid3D g( 80, 80, 80 );
	//-- Put the obstacles in the borders
    g.CreateExternalBoundary();
	g.CreateBox( 20, 20, 10, 10, 40, 60 );
	g.CreateBox( 50, 20, 10, 10, 40, 60 );

	//-- See how it looks
	g.Visualization();

	//-- Create its DT
	printf(" Creating Topology 3D guy \n");
    TopologyGrid3D tg3d( &g );
	printf(" Start DT \n");
	tg3d.CalculateDT();
	printf(" End DT \n");

	//-- Show ridge
	tg3d.VisualizeRidge();

	//-- Create a Search object

	gRidge = tg3d.GetDTRidge();	
	mSearch = new Search( &gRidge );

	printf(" Start search \n");
	int n = gRidge.size() - 1;
	Pos start = gRidge[0].pos;
	Pos goal = gRidge[n].pos;
	std::vector< std::vector<Pos> >  paths = mSearch->FindDiversePaths( start, goal, 3 );

	std::vector< std::vector<Pos> >  paths = mSearch->FindDiversePaths( gRidge[0].pos.x, gRidge[0].pos.y, gRidge[0].pos.z, gRidge[n].pos.x, gRidge[n].pos.y, gRidge[n].pos.z, 3 );

	printf("End Search \n");
	DrawResult( &g, paths );


    return 0;
	
}

/**
 * @function DrawPath
 */
void DrawResult( Grid3D *_g, std::vector< std::vector<Pos> >  _paths ) {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects( new pcl::PointCloud<pcl::PointXYZRGB> );

	objects->height = 1;
	objects->is_dense = false;
	objects->points.resize( 0 );

	uint8_t r = 0; 
	uint8_t g =  0; 
	uint8_t b = 255;			
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

	pcl::PointXYZRGB q;

	//-- Obstacles		
	for( unsigned int i = 0; i < _g->GetSizeX(); i++ ) {	
		for( unsigned int j = 0; j < _g->GetSizeY(); j++ ) {
			for( unsigned int k = 0; k < _g->GetSizeZ(); ++k ) {
				if( _g->GetState( i, j, k ) == true ) { // Obstacle
					q.x = (double) i;
					q.y = (double) j;
					q.z = (double) k;				
					q.rgb = *reinterpret_cast<float*>(&rgb);
					objects->points.push_back(q);       
				}
			}
		}
	}


	r = 255; g =  0; b = 0;			
    rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

	//-- Path
	for( int i = 0; i < _paths.size(); ++i ) {
		printf("Size Path [%d]: %d \n", i, _paths[i].size() );
		for( int j = 0; j < _paths[i].size(); ++j ) {
			q.x = _paths[i][j].x;
			q.y = _paths[i][j].y;
			q.z = _paths[i][j].z;
			q.rgb = *reinterpret_cast<float*>(&rgb);
			objects->points.push_back(q);
		}  		
	}

	pcl::visualization::CloudViewer viewer( "Path Visualization" );
	viewer.showCloud( objects );
	while( !viewer.wasStopped() ) {		
	}	
		

}



