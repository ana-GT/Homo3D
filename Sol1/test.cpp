/**
 * @file test.cpp
 */
#include "distance_field/include/distance_field/voxel_grid.h"
#include "distance_field/include/distance_field/distance_field.h"
#include "distance_field/include/distance_field/pf_distance_field.h"
#include "distance_field/include/distance_field/collision_map.h"
#include "HomoSearch.h"
#include <string>
#include <stdio.h>


using namespace distance_field;

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

	CollisionMap *cm = new CollisionMap( 160, 140, 50, 1, 0.0, 0.0, 0.0, ERROR );
	bool st;
	try {
		cm->addBox( 10, 30, 10, 40, 70, 20 );
		cm->addBox( 20, 40, 10, 80, 60, 30 );
		cm->addBox( 40, 20, 20, 30, 110, 20 );
		cm->addBox( 40, 10, 10, 30, 30, 30 );
		cm->addExternalBoundary();

	} catch( std::string error ) {
		std::cout<< "-->" << error << std::endl;
	}

    cm->visualize();

	//-- Create the distance transform
	PFDistanceField p( 160, 140, 50, 1, 0.0, 0.0, 0.0 );
	p.addCollisionMapToField( cm );

    std::vector<Eigen::Vector3i> m = p.getRidgePoints();
    printf("Number of ridge points: %d \n", m.size() );

	//p.visualizeDT();
	

	//-- Homo search
	HomoSearch hs( cm, &m );
    hs.searchRidge();
	std::vector<Eigen::Vector3i> path = hs.mPath;

    //-- Draw ridge
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects( new pcl::PointCloud<pcl::PointXYZRGB> );

	objects->height = 1;
	objects->is_dense = false;
	objects->points.resize( 0 );

	uint8_t r = 255; uint8_t g =  0; uint8_t b = 0;			
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

    int count = 0;
	//-- First, find min and max distances
		
	for( size_t i = 0; i < m.size() ; ++i ) {

		pcl::PointXYZRGB q;
        double wx; double wy; double wz;
        p.gridToWorld( m[i](0), m[i](1), m[i](2), wx, wy, wz );
		q.x = wx; q.y = wy; q.z = wz;					
		q.rgb = *reinterpret_cast<float*>(&rgb);
		objects->points.push_back(q); count++;        
	}

	for( size_t i = 0; i < cm->getNumCells(VoxelGrid<VoxelState>::DIM_X) ; ++i ) {
		for( size_t j = 0; j < cm->getNumCells(VoxelGrid<VoxelState>::DIM_Y) ; ++j ) {
				for( size_t k = 0; k < cm->getNumCells(VoxelGrid<VoxelState>::DIM_Z) ; ++k ) {

					if( cm->getCell(i,j,k) == OCCUPIED ) {
						pcl::PointXYZRGB q;
			            double wx; double wy; double wz;
			            p.gridToWorld( i, j, k, wx, wy, wz );
						q.x = wx; q.y = wy; q.z = wz;					
						uint8_t r = 0; uint8_t g =  0; uint8_t b = 255;			
			    		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		
						q.rgb = *reinterpret_cast<float*>(&rgb);
						objects->points.push_back(q);
					}
				}
		}
	}


	for( size_t i = 0; i < path.size() ; ++i ) {

		pcl::PointXYZRGB q;
        double wx; double wy; double wz;
        p.gridToWorld( path[i](0), path[i](1), path[i](2), wx, wy, wz );
		q.x = wx; q.y = wy; q.z = wz;	
		uint8_t r = 0; uint8_t g =  255; uint8_t b = 255;			
		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);				
		q.rgb = *reinterpret_cast<float*>(&rgb);
		objects->points.push_back(q); count++;        
	}



	pcl::visualization::CloudViewer viewer( "Gx visualization" );
	viewer.showCloud( objects );
	while( !viewer.wasStopped() ) {		
	}	


    printf("Deleting");
    delete cm;

	return 0;
}
