/**
 * @file test.cpp
 */

//-- PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <time.h>

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

	srand( time(NULL) );

	//-- Create grid
    Grid3D g( 80, 80, 80 );
	//-- Put the obstacles in the borders
    g.CreateExternalBoundary();
	g.CreateBox( 20, 20, 10, 10, 40, 60 );
	g.CreateBox( 50, 20, 10, 10, 40, 60 );
	//g.CreateBox( 20, 20, 30, 40, 40, 20 );

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
	std::vector<Cell> free = tg3d.GetFreeCells(); 
	mSearch = new Search( &free );
	//mSearch = new Search( &gRidge );
	printf(" Start search \n");
	int n = gRidge.size() - 1;

	Pos start; start.x = 10, start.y = 10; start.z = 40;
	Pos goal; goal.x = 70, goal.y = 70; goal.z = 40;
	//Pos start; start.x = 40, start.y = 50; start.z = 25;
	//Pos goal; goal.x = 40, goal.y = 10; goal.z = 55;

	std::vector< std::vector<Pos> >  paths = mSearch->FindDiversePaths( start, goal, 4 );


	printf("End Search \n");
	DrawResult( &g, paths );


    return 0;
	
}

/*
void DrawGrid3D( pcl::visualization::PCLVisualizer *_viewer ) {
	viewer->addCloud
}
*/
/**
 * @function DrawPath
 */
void DrawResult( Grid3D *_g, std::vector< std::vector<Pos> >  _paths ) {

	//-- Obstacles
	pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud( new pcl::PointCloud<pcl::PointXYZ> );

	objectsCloud->height = 1;
	objectsCloud->is_dense = false;
	objectsCloud->points.resize( 0 );

	pcl::PointXYZ q;
		
	for( unsigned int i = 0; i < _g->GetSizeX(); i++ ) {	
		for( unsigned int j = 0; j < _g->GetSizeY(); j++ ) {
			for( unsigned int k = 0; k < _g->GetSizeZ(); ++k ) {
				if( _g->GetState( i, j, k ) == true ) { // Obstacle
					q.x = (double) i; q.y = (double) j; q.z = (double) k;				
					objectsCloud->points.push_back(q);       
				}
			}
		}
	}

	//-- Paths
	int numPaths = _paths.size();
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pathCloud;

	for( size_t i = 0; i < numPaths; ++i ) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc( new pcl::PointCloud<pcl::PointXYZ> );
		printf("Size Path [%d]: %d \n", i, _paths[i].size() );
		for( int j = 0; j < _paths[i].size(); ++j ) {
			q.x = _paths[i][j].x;
			q.y = _paths[i][j].y;
			q.z = _paths[i][j].z;
			pc->points.push_back(q);
		}  		
		pathCloud.push_back( pc );
	}


	pcl::visualization::PCLVisualizer *viewer;
	viewer = new pcl::visualization::PCLVisualizer( "Test PCL" );

	viewer->setBackgroundColor( 0.0, 0.0, 0.0 );
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> objectsColor( objectsCloud, 0, 0, 255 );
	viewer->addPointCloud<pcl::PointXYZ>(objectsCloud, objectsColor, "Objects Cloud");


	for( size_t i = 0; i < numPaths; ++i ) {
		double r; double g; double b;
		r = ( rand() % 256 )/255.0;
		g = ( rand() % 256 )/255.0;
		b = ( rand() % 256 )/255.0;
		for( int j = 0; j < pathCloud[i]->points.size() - 1; ++j ) {

		char linename[12];	
		sprintf( linename, "line%d-%d", i,j );
		std::string id(linename);

		viewer->addLine<pcl::PointXYZ>( pathCloud[i]->points[j], pathCloud[i]->points[j + 1], r, g, b, id );
		}
	}
	
	while( !viewer->wasStopped() ) {
		viewer->spinOnce(100);
	}
		

}

