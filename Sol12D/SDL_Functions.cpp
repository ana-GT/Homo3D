/**
 * @file SDL_Functions.cpp
 */
#include "testSDL.h"

/**
 * @function GetMousePos
 */
Pos GetMousePos( int _x, int _y ) {
	Pos p;
	p.x = (int) ( _x / TILE_WIDTH );
	p.y = (int) ( _y / TILE_HEIGHT );
	return p;
}

/**
 * @function DrawPath
 */
void DrawPath( int _index ) {

	///  Draw paths
	for( unsigned int j = 0; j < gPaths[_index].size(); ++j ) {
		apply_surface( TILE_WIDTH*(gPaths[_index][j].x), TILE_HEIGHT*(gPaths[_index][j].y), paths, screen, &clipsPaths[(_index % 9)*0] );	
	}

}

/**
 * @function DrawRidge
 */
void DrawRidge() {

	///  Draw paths
	for( unsigned int j = 0; j < gRidge.size(); ++j ) {
		apply_surface( TILE_WIDTH*( gRidge[j].pos.x), TILE_HEIGHT*( gRidge[j].pos.y), paths, screen, &clipsPaths[8] );	
	}

}


/**
 * @function DrawScene
 */
void DrawScene( Grid2D *_g ) {
	
	for( unsigned int i = 0; i < _g->GetSizeX(); ++i ) {
		for( unsigned j = 0; j < _g->GetSizeY(); ++j ) {
			if( _g->GetState( i, j ) == false ) { // free
				if( (i + j) % 3 == 0 ) {
					apply_surface( TILE_WIDTH*i, TILE_HEIGHT*j, tiles, screen, &clips[TILE_RED] ); }
				else if( (i + j) % 3 == 1 ) {
					apply_surface( TILE_WIDTH*i, TILE_HEIGHT*j, tiles, screen, &clips[TILE_GREEN] ); }
				if( (i + j) % 3 == 2 ) {
					apply_surface( TILE_WIDTH*i, TILE_HEIGHT*j, tiles, screen, &clips[TILE_BLUE] ); }

			}

			else {
				apply_surface( TILE_WIDTH*i, TILE_HEIGHT*j, tiles, screen, &clips[TILE_BLACK] );
			}
		}
	}

	/// Draw Ridge
	if( gRidge.size() > 0 ) {
		DrawRidge();
	} 
	
	/// Draw Path
	if( numPaths > 0 &&  countPath >= 0 ) {
		DrawPath( countPath );
	}


	/// Draw Start and Goal
	if( mStart.x >= 0 && mStart.y >= 0 ) {
		apply_surface( TILE_WIDTH*(mStart.x), TILE_HEIGHT*(mStart.y), v0, screen, &clips[TILE_V0] );		
	}
	if( mGoal.x >= 0 && mGoal.y >= 0 ) {
		apply_surface( TILE_WIDTH*(mGoal.x), TILE_HEIGHT*(mGoal.y), v0, screen, &clips[TILE_V0] );		
	}


	/// Update the screen
	if( SDL_Flip( screen ) == -1 ) {
		printf( "--(!) SDL_Flip not worked correctly\n" );
	}
}


/**
 * @function load_files
 */
bool load_files() {

	/// Load the image
	tiles = load_image( "images/tiles.png" );
	v0 = load_image( "images/V0.png" );
	paths = load_image( "images/paths.png" );

	/// If there was an error in loading 
	if( tiles == NULL || v0 == NULL || paths == NULL ) {
		return false;
	}

	/// If everything loaded fine
	return true;

}

/**
 * @function clean_up
 */
void clean_up() {
	/// Free the image
	SDL_FreeSurface( tiles );
	SDL_FreeSurface( v0 );
	SDL_FreeSurface( paths );

	/// Quit SDL
	SDL_Quit();
}

/**
 * @function load_image
 */
SDL_Surface *load_image( std::string filename ){

	/// Temporary storage for the image that is loaded
	SDL_Surface* loadedImage = NULL;

	/// The optimized image that will be used
	SDL_Surface* optimizedImage = NULL;

	/// Load the image
	loadedImage = IMG_Load( filename.c_str() );

	if( loadedImage != NULL );
	{
		/// Create an optimized image
		optimizedImage = SDL_DisplayFormat( loadedImage );
	
		/// Free the old image
		SDL_FreeSurface( loadedImage );

		/// If the image was optimized just fine
		if( optimizedImage != NULL ) {
			/// Map the color key	
			Uint32 colorkey = SDL_MapRGB( optimizedImage->format, 0x00, 0xFF, 0xFF );

			/// Set all pixels of color R 0, G 0xFF, B 0xFF to be transparent
			SDL_SetColorKey( optimizedImage, SDL_SRCCOLORKEY, colorkey );
		}

		/// Return the optimized image
		return optimizedImage;
	}
}

/**
 * @function apply_surface
 */
void apply_surface( int x, int y, 
					SDL_Surface* source, 
					SDL_Surface *destination,
					SDL_Rect *clip ) {

	/// Hold the offsets
	SDL_Rect offset;

	/// Get offsets
	offset.x = x;
	offset.y = y;

	/// Blit the surface
	SDL_BlitSurface( source, clip, destination, &offset );
	
}

/**
 * @function clip_tiles
 */
void clip_tiles() {
    clips[ TILE_RED ].x = 0;
    clips[ TILE_RED ].y = 0;
    clips[ TILE_RED ].w = TILE_WIDTH;
    clips[ TILE_RED ].h = TILE_HEIGHT;

    clips[ TILE_GREEN ].x = 0;
    clips[ TILE_GREEN ].y = TILE_HEIGHT*1;
    clips[ TILE_GREEN ].w = TILE_WIDTH;
    clips[ TILE_GREEN ].h = TILE_HEIGHT;

    clips[ TILE_BLUE ].x = 0;
    clips[ TILE_BLUE ].y = TILE_HEIGHT*2;
    clips[ TILE_BLUE ].w = TILE_WIDTH;
    clips[ TILE_BLUE ].h = TILE_HEIGHT;

    clips[ TILE_BLACK ].x = TILE_WIDTH*2;
    clips[ TILE_BLACK ].y = TILE_HEIGHT*1;
    clips[ TILE_BLACK ].w = TILE_WIDTH;
    clips[ TILE_BLACK ].h = TILE_HEIGHT;

    clips[ TILE_V0 ].x = 0;
    clips[ TILE_V0 ].y = 0;
    clips[ TILE_V0 ].w = TILE_WIDTH;
    clips[ TILE_V0 ].h = TILE_HEIGHT;

	int ind = 0;

	for( int i = 0; i < 3; ++i ) {
		for( int j = 0; j < 3; ++j ) {
		    clipsPaths[ind].x = i*TILE_WIDTH;
		    clipsPaths[ind].y = j*TILE_HEIGHT;
		    clipsPaths[ind].w = TILE_WIDTH;
		    clipsPaths[ind].h = TILE_HEIGHT;
			++ind;			
		}
	}

}

/**
 * @function display_options
 */
void display_options() {
	printf( "--(*) OPTIONS (*) \n" );
	printf( "==================\n" );
	printf( "--(*) Press [o] to set obstacles \n" );
	printf( "--(*) Press [v] to create Voronoi guy \n" );
	printf( "--(*) Press [p] to print DT \n" );
	printf( "--(*) Press [s] to pick the start \n" );
	printf( "--(*) Press [g] to pick the goal \n" );
	printf( "--(*) Press [x] to do the search \n" );
	printf( "--(*) Press [c] to check out the paths \n" );
	printf( "--(*) Press [n] to check the neighbors of the clicked cell \n" );
	printf( "\n" );
}
