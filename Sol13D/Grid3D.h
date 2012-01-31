/**
 * @file Grid3D.h
 */

#ifndef _HP3D_GRID_3D_H_
#define _HP3D_GRID_3D_H_

#include <vector>

/**
 * @struct Pos
 */
struct Pos {
	int x;
	int y;
	int z;
};

/**
 * @class Grid3D
 */
class Grid3D {

public:
	int mSizeX;
	int mSizeY;
	int mSizeZ;
	int mSize;

	int mStride1;
	int mStride2;

	/// Functions
	Grid3D( int _sizeX, int _sizeY, int _sizeZ );
    ~Grid3D();
	void SetState( Pos _p, bool _state );
	void SetState( int _x, int _y, int _z, bool _state );
	int ref( Pos _p ) const;
	int ref( int _x, int _y, int _z ) const;
	bool GetState( Pos _p ) const;
	bool GetState( int _x, int _y, int _z ) const;
	int GetSizeX() const;
	int GetSizeY() const;
	int GetSizeZ() const;
	bool CheckCollision( Pos _p ) const;

	//-- Utility functions
	void Visualization();
	void CreateExternalBoundary();
	void CreateBox( int _x, int _y, int _z, int _dimX, int _dimY, int _dimZ );

private:
	bool* mState;
};


/////////// INLINE FUNCTIONS ////////////////////////

inline int Grid3D::GetSizeX() const {
	return mSizeX;
}

inline int Grid3D::GetSizeY() const {
	return mSizeY;
}

inline int Grid3D::GetSizeZ() const {
	return mSizeZ;
}

#endif /** _HP2D_GRID_3D_H_  */
