/**
 * @file Grid2D.h
 */

#ifndef _HP2D_GRID_2D_H_
#define _HP2D_GRID_2D_H_

#include <vector>

/**
 * @struct Pos
 */
struct Pos {
	int x;
	int y;
};

/**
 * @class Grid2D
 */
class Grid2D {

public:
	int mSizeX;
	int mSizeY;
	int mSize;

	/// Functions
	Grid2D( int _sizeX, int _sizeY );
    ~Grid2D();
	void SetState( Pos _p, bool _state );
	int ref( Pos _p ) const;
	int ref( int _x, int _y ) const;
	bool GetState( Pos _p ) const;
	bool GetState( int _x, int _y ) const;
	int GetSizeX() const;
	int GetSizeY() const;
	bool CheckCollision( Pos _p ) const;

private:
	bool* mState;
};


/////////// INLINE FUNCTIONS ////////////////////////

inline int Grid2D::GetSizeX() const {
	return mSizeX;
}

inline int Grid2D::GetSizeY() const {
	return mSizeY;
}

#endif /** _HP2D_GRID_2D_H_  */
