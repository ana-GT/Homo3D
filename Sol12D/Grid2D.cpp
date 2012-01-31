/**
 * @function Grid2D.cpp
 * @brief Functions for Grid2D.cpp
 */

#include "Grid2D.h"

/**
 * @function Grid2D
 * @brief Constructor	
 */
Grid2D::Grid2D( int _sizeX, int _sizeY ) {

	mSizeX = _sizeX;
	mSizeY = _sizeY;
    mSize = mSizeX*mSizeY;
	mState = new bool[mSize]; 

    /// Set all cells to free: false
	for( unsigned int j = 0; j < mSizeY; j++ ) {
		for( unsigned int i = 0; i < mSizeX; i++ ) {
			Pos p; 
            p.x = i; p.y = j;
			SetState( p, false );
		}
	}	

}


/**
 * @function Grid2D
 * @brief Destructor	
 */
Grid2D::~Grid2D() {
	delete [] mState;
}

/**
 * @function GetState
 */
bool Grid2D::GetState( Pos _p ) const {
	return mState[ ref(_p) ];
}

/**
 * @function GetState
 */
bool Grid2D::GetState( int _x, int _y ) const {
	return mState[ ref(_x, _y) ];
}

/**
 * @function CheckCollision
 */
bool Grid2D::CheckCollision( Pos _p ) const {
	if( GetState( _p ) == false ) { // free
		return false;
	}
	else { // occupied
		return true;
	}
}


/**
 * @function SetCell
 */
void Grid2D::SetState( Pos _p, bool _state ) {
	mState[ref(_p)] = _state;
}

/**
 * @function ref
 */
inline int Grid2D::ref( Pos _p ) const {
	return (_p.x)*mSizeY + _p.y;
}

/**
 * @function ref
 */
int Grid2D::ref( int _x, int _y ) const {
	return _x*mSizeY + _y;
}
