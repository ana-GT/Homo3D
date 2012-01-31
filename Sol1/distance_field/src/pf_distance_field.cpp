/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Intel Labs Pittsburgh
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Intel Labs nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** 
 * @author Siddhartha Srinivasa 
 * @brief
 */

#include <distance_field/pf_distance_field.h>
#include <algorithm>
#include <limits>


namespace distance_field
{

const int PFDistanceField::NeighborVertex[8][3] = { {-1,-1,1}, {-1,1,1}, {1,1,1}, {1,-1,1}, {-1,-1,-1}, {-1,1,-1}, {1,1,-1}, {1,-1,-1} };
const int PFDistanceField::NeighborEdge[12][3] = { {-1,-1,0}, {-1,1,0}, {1,1,0}, {1,-1,0}, {0,-1,1}, {-1,0,1}, {0,1,1}, {1,0,1}, {0,-1,-1}, {-1,0,-1}, {0,1,-1}, {1,0,-1} };
const int PFDistanceField::NeighborFace[6][3] =  { {-1,0,0}, {0,1,0}, {1,0,0}, {0,-1,0}, {0,0,1}, {0,0,-1} };


/**
 * @function PFDistanceField 
 * @brief Constructor
 */
PFDistanceField::PFDistanceField( double size_x, 
								  double size_y, 
								  double size_z, 
								  double resolution,
    							  double origin_x, 
								  double origin_y, 
								  double origin_z):
  DistanceField<float>( size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, DT_INF ),
  DT_INF( std::numeric_limits<float>::max() )
{
   reset();
}

/**
 * @function ~PFDistanceField
 * @brief Destructor
 */
PFDistanceField::~PFDistanceField()
{
}

/**
 * @function getRidgePoints
 * @brief Get Local maxima from DT
 */
std::vector<Eigen::Vector3i> PFDistanceField::getRidgePoints() {

	std::vector<Eigen::Vector3i> ridgePoints;
	for ( size_t i = 0; i < getNumCells( DIM_X ); ++i ) {
    	for ( size_t j = 0; j < getNumCells( DIM_Y ); ++j ) {
	    	for ( size_t k = 0; k < getNumCells( DIM_Z ); ++k ) {			
			
				if( DTLocalMaxima( i, j, k ) == true ) {
					ridgePoints.push_back( Eigen::Vector3i(i,j,k) );
				}			
			}
		}
	}

	return ridgePoints;
}



/**
 * @function DTLocalMaxima
 */
bool PFDistanceField::DTLocalMaxima( int _x, int _y, int _z ) {

	double tune = 1.0;
	double dist = getDistanceFromCell( _x, _y, _z );

	if( dist == 0.0 ) { return false; } // It is an obstacle, NOT ridge

	//-- Check the Vertex neighbors
	for( int ind = 0; ind < 8; ++ind ) {

		int nx = _x + NeighborVertex[ind][0];		
		int ny = _y + NeighborVertex[ind][1];		
		int nz = _z + NeighborVertex[ind][2];

		if( isCellValid( nx, ny, nz ) == false ) { continue; }		

		if( getDistanceFromCell(nx, ny, nz) > dist  + resolution_[0]*1.71*tune ) {
			return false;
		}   

	}

	//-- Check the Edge Neighbors
	for( size_t ind = 0; ind < 12; ++ind ) {

		int nx = _x + NeighborEdge[ind][0];		
		int ny = _y + NeighborEdge[ind][1];		
		int nz = _z + NeighborEdge[ind][2];

		if( isCellValid( nx, ny, nz ) == false ) { continue; }		

		if( getDistanceFromCell(nx, ny, nz) > dist + resolution_[0]*0.95*tune ) {
			return false;
		}   
	}

	//-- Check the Face neighbors
	for( size_t ind = 0; ind < 6; ++ind ) {

		int nx = _x + NeighborFace[ind][0];		
		int ny = _y + NeighborFace[ind][1];		
		int nz = _z + NeighborFace[ind][2];

		if( isCellValid( nx, ny, nz ) == false ) { continue; }		

		if( getDistanceFromCell(nx, ny, nz) > dist + resolution_[0]*1.0*tune ) {
			return false;
		}   
	}

	//-- Otherwise, we have got a ridge! :)
	return true; 

}

/**
 * @function addPointsToField
 * @brief 
 */
void PFDistanceField::addPointsToField( const std::vector<Eigen::Vector3d> points )
{
  int x, y, z;
  float init = 0.0;

  for (unsigned int i=0; i<points.size(); ++i)
  {
    bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
    if (!valid)
      continue;
   
    setCell( x, y, z, init );

  }

  computeDT();
}

/**
 * @function computeDT
 * @brief Compute Distance Transform
 */
void PFDistanceField::computeDT()
{
  printf("Computing DT \n");
  size_t nx = num_cells_[DIM_X];
  size_t ny = num_cells_[DIM_Y];
  size_t nz = num_cells_[DIM_Z];

    size_t     maxdim = std::max( nx, std::max(ny, nz) );
    FloatArray f(maxdim), ft(maxdim), zz(maxdim+1);
    IntArray   v(maxdim);

    // along z
    for (size_t y=0; y<ny; ++y) {
        for (size_t x=0; x<nx; ++x) {
            for (size_t z=0; z<nz; ++z) {
                f[z] = getCell(x,y,z);
            }
            dt(f, nz, ft, v, zz);
            for (size_t z=0; z<nz; ++z) {
                setCell(x,y,z,ft[z]);
            }
        }
    }

    // along y
    for (size_t z=0; z<nz; ++z) {
        for (size_t x=0; x<nx; ++x) {
            for (size_t y=0; y<ny; ++y) {
                f[y] = getCell(x,y,z);
            }
            dt(f, ny, ft, v, zz);
            for (size_t y=0; y<ny; ++y) {
                setCell(x,y,z,ft[y]);
            }
        }
    }

    // along x
    for (size_t z=0; z<nz; ++z) {
        for (size_t y=0; y<ny; ++y) {
            for (size_t x=0; x<nx; ++x) {
                f[x] = getCell(x,y,z);
            }
            dt(f, nx, ft, v, zz);
            for (size_t x=0; x<nx; ++x) {
                setCell(x,y,z,ft[x]);
            }
        }
    }

}

/**
 * @function dt
 * @brief Get DT
 */
void PFDistanceField::dt(const FloatArray& f,
        size_t nn,
        FloatArray& ft,
        IntArray& v,
        FloatArray& z) {

/*    assert(f.size() >= nn);
    assert(ft.size() >= nn);
    assert(v.size() >= nn);
    assert(z.size() >= nn+1);
*/
    int n = nn;

    int k = 0;

    v[0] = 0;

    z[0] = -DT_INF;
    z[1] =  DT_INF;

    for (int q=1; q<n; ++q) {
        float s = ((f[q]+sqr(q))-(f[v[k]]+sqr(v[k])))/(2*q-2*v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q]+sqr(q))-(f[v[k]]+sqr(v[k])))/(2*q-2*v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k+1] = DT_INF;
    }

    k = 0;
    for (int q=0; q<n; ++q) {
        while (z[k+1] < q) { ++k; }
        ft[q] = sqr(q-v[k]) + f[v[k]];
    }

}

/**
 * function reset
 * @brief Reset all elements to infinite
 */
void PFDistanceField::reset()
{
  VoxelGrid<float>::reset(DT_INF);
} 

/**
 * @function compute Gradient
 */
VoxelGrid<Eigen::Vector3d> PFDistanceField::computeGradient() {

	VoxelGrid<Eigen::Vector3d> grad( getSize(VoxelGrid<float>::DIM_X), 
							         getSize(VoxelGrid<float>::DIM_Y), 
									 getSize(VoxelGrid<float>::DIM_Z),
                                     getResolution(VoxelGrid<float>::DIM_X),
     							     getOrigin(VoxelGrid<float>::DIM_X), 
                                     getOrigin(VoxelGrid<float>::DIM_Y),
                                     getOrigin(VoxelGrid<float>::DIM_Z),
                                     Eigen::Vector3d(8.0, 8.0, 8.0) );


    double wx; double wy; double wz;
    double gx; double gy; double gz;

    int nt = 0; int nx  = 0; int ny = 0; int nz = 0;

	for ( size_t i = 0; i < getNumCells( VoxelGrid<float>::DIM_X ); ++i ) {
    	for ( size_t j = 0; j < getNumCells( VoxelGrid<float>::DIM_Y ); ++j ) {
	    	for ( size_t k = 0; k < getNumCells( VoxelGrid<float>::DIM_Z ); ++k ) {	
 
                    gridToWorld( i, j, k, wx, wy, wz );
					getDistanceGradient(wx, wy, wz, gx, gy, gz);

                    if ( gx == 0.0 &&  gy == 0.0 && gz == 0.0 ) { nt++; } 
                    if( gx == 0.0 ) { nx++; }
                    if( gy == 0.0 ) { ny++; }
                    if( gz == 0.0 ) { nz++; }

                    Eigen::Vector3d g; g(0) = gx; g(1) = gy; g(2) = gz; 
					grad.setCell( i, j, k, g );		
			}		
	 	}	
   }

    printf("Computed gradient xyz0: %d x0: %d y0: %d z0: %d \n", nt, nx, ny, nz );

	return grad;

}

/**
 * @function visualizeDT
 */
void PFDistanceField::visualizeDT() {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects( new pcl::PointCloud<pcl::PointXYZRGB> );

	//-- Load 3D environment info of objects into a PointCloud

	objects->width = num_cells_total_;
	objects->height = 1;
	objects->is_dense = false;
	objects->points.resize( 0 );

	
    double m = 100000; double M=-1200;
    uint8_t rm = 255; uint8_t rM = 0; 

	//-- First, find min and max distances
	for( size_t i = 0; i < getNumCells(DIM_X) ; ++i ) {
		for( size_t j = 0; j < getNumCells(DIM_Y); ++j ) {
			for( size_t k = 0; k < getNumCells(DIM_Z); ++k ) {

				double dist = getDistanceFromCell( i, j, k );
				if( dist < m ) { m = dist; }
                if( dist > M ) { M = dist; }
			}
		}
	}

    printf("Min Dist: %f Max Dist: %f \n", m, M );

	//-- Set appropiate colors
	for( size_t i = 0; i < getNumCells(DIM_X) ; ++i ) {
		for( size_t j = 0; j < getNumCells(DIM_Y); ++j ) {
			for( size_t k = 0; k < getNumCells(DIM_Z); ++k ) {

				pcl::PointXYZRGB p;
                double wx; double wy; double wz;
                gridToWorld( i, j, k, wx, wy, wz );
				p.x = wx; p.y = wy; p.z = wz;
					

				double dist = getDistanceFromCell( i, j, k );
				//-- Color
				uint8_t r = (int)( ( 255.0/(M-m) )*( dist - m ) ); 
                uint8_t g = (int)( ( 255.0/(M-m) )*( dist - m ) ); 
                uint8_t b = (int)( ( 255.0/(M-m) )*( dist - m ) );			
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

				if( r < rm ) { rm = r; }
                if( r > rM ) { rM = r; }

				p.rgb = *reinterpret_cast<float*>(&rgb);
				objects->points.push_back(p);
			}
		}
		
	}
    printf("Min max: %f %f r: %d %d \n", m,M, rm, rM);
	pcl::visualization::CloudViewer viewer( "DT visualization" );
	viewer.showCloud( objects );
	while( !viewer.wasStopped() ) {
		
	}


}

/**
 * @function
 visualizeObjects
 */
void PFDistanceField::visualizeObjects() {
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects( new pcl::PointCloud<pcl::PointXYZRGB> );

	//-- Load 3D environment info of objects into a PointCloud

	objects->width = num_cells_total_;
	objects->height = 1;
	objects->is_dense = false;
	objects->points.resize( 0 );

	int index = 0;

	//-- Define color for objects - Green
	uint8_t r = 0; uint8_t g = 255; uint8_t b = 0;
	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	

	//-- Set the occupied cells
	for( size_t i = 0; i < num_cells_[VoxelGrid<float>::DIM_X] ; ++i ) {
		for( size_t j = 0; j < num_cells_[VoxelGrid<float>::DIM_Y]; ++j ) {
			for( size_t k = 0; k < num_cells_[VoxelGrid<float>::DIM_Z]; ++k ) {
				if( getCell( i, j, k ) == 0.0 ) {
					pcl::PointXYZRGB p;
					p.x = getLocationFromCell( DIM_X, i );
					p.y = getLocationFromCell( DIM_Y, j );
					p.z = getLocationFromCell( DIM_Z, k );
					p.rgb = *reinterpret_cast<float*>(&rgb);
					objects->points.push_back(p);
				    ++index;
				}
			}
		}
		
	}
    printf("Num points: %d \n", index);
	pcl::visualization::CloudViewer viewer( "Objects visualization" );
	viewer.showCloud( objects );
	while( !viewer.wasStopped() ) {
		
	}

}


} /** end of namespace distance_field */
