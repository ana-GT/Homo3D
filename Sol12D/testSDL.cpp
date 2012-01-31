/**
 * @file testSDL.cpp
 */

#include "testSDL.h"

//////////// Extern Variables /////////////////////

//-- Surfaces
SDL_Surface *screen = NULL;
SDL_Surface *tiles = NULL;
SDL_Surface *v0 = NULL;
SDL_Surface *paths = NULL;

//-- Attributes of the screen
int SCREEN_WIDTH;
int SCREEN_HEIGHT;
const int SCREEN_BPP = 32;

//-- Tile constants
const int TILE_WIDTH = 40;
const int TILE_HEIGHT = 40;
const int TILE_SPRITES = 5;

//-- Sprites
const int TILE_RED = 0;
const int TILE_GREEN = 1;
const int TILE_BLUE = 2;
const int TILE_BLACK = 3;
const int TILE_V0 = 4;

SDL_Rect clips[TILE_SPRITES];
SDL_Rect clipsPaths[9]; // I set this 

//-- Event structure
SDL_Event event;

//-- Global variables
int SIZE_X;
int SIZE_Y;
int MOUSE_MODE;
Pos mStart;
Pos mGoal;
Search *mSearch;;
std::vector< std::vector<Pos> > gPaths;
std::vector<Cell> gRidge;
int numPaths;
int countPath;
/////////////////////////////////////////////////


/**
 * @function main  
 */
int main( int argc, char *argv[] ) {

	bool quit = false;
	countPath = 0;

	//-- Initialize
	if( init() == false ) {
		return 1;
	}

	//-- Load the files
	if( load_files() == false ) {
		return 1;
	}

	//-- Clip tiles
	clip_tiles();

	//-- Create grid
    Grid2D g( SIZE_X, SIZE_Y );
	//-- Put the obstacles in the borders
    createBorderingObstacles( &g );

	//-- Create HomoPath object
    TopologyGrid2D tg2d( &g );


	//-- Display options
	display_options();

	//-- Start SDL
	while( quit == false ) {
		while( SDL_PollEvent( &event ) ) {

			if( event.type == SDL_QUIT ) {
				quit = true;
			}
			else if( event.type == SDL_KEYDOWN ) {
				switch( event.key.keysym.sym ) {
					case 'v': {
						printf("-- Calculating approximate Voronoi of 2D Map \n");
						calculateDT( &tg2d );
						break;
					}
					case 'o': {
						printf("-- Click one location to be obstacle \n");
						MOUSE_MODE = 0;
						break;
					}
					case 's': {
						printf("-- Click one location to be the Start \n");
						MOUSE_MODE = 1;
						break;
					}

					case 'g': {
						printf("-- Click one location to be the Goal \n");
						MOUSE_MODE = 2;
						break;
					}

					case 'n': {
						printf("-- Checking this grid's neighbors \n");
						MOUSE_MODE = 3;
						break;
					}

					case 'x': {
						printf("-- Doing the search \n");
						//mSearch = new Search( &gRidge );
						mSearch  = new Search( &( tg2d.GetFreeCells() ) );
						gPaths =  mSearch->FindDiversePaths( mStart.x, mStart.y, mGoal.x, mGoal.y, 15 );
						numPaths = gPaths.size();
						countPath = 0;
						//printPaths();
						break;
					}
					case 'c': {
						countPath++;
						countPath = ( countPath % numPaths );
						printf( "-- Checking path [%d] \n", countPath );
						break;
					}

					case 'p': {
						printf("-- Printing DT \n");
						tg2d.PrintDT();
					}
					case 'q': {
						display_options();
					}
				}
			}

			else if( event.type == SDL_MOUSEBUTTONDOWN ) {
				if( event.button.button == SDL_BUTTON_LEFT ) {

					int x = event.button.x;
					int y = event.button.y;					
					Pos p = GetMousePos( x, y );

					switch( MOUSE_MODE ) {
						case 0: {
							printf("--> Obstacle located at (%d, %d) \n", p.x, p.y );
							if( g.GetState(p) == false ) { 	
								g.SetState( p, true ); 
							}
							else {
								g.SetState( p, false );
							}
							printf("--* To see options again press [q] \n");			
							break;
						}

						case 1: {
							printf("--> Start located at (%d %d) \n", p.x, p.y );
							mStart = p;
							printf("--* To see options again press [q] \n");
							break;
						}

						case 2: {
							printf("--> Goal located at (%d %d) \n", p.x, p.y );
							mGoal = p;
							printf("--* To see options again press [q] \n");
							break;
						}

						case 3: {
							printf("--> Checking neighbors at (%d %d) \n", p.x, p.y );
							mSearch->CheckNeighbors( (double) p.x, (double) p.y );
							printf("--* To see options again press [q] \n");
							break;
						}

					}

				}
			}

		//-- Draw
		DrawScene( &g );

		} // end while SDL_PollEvent
		

	} // end while quit

	clean_up();
    return 0;
	
}


/**
 * @function CallPlanner
 */
void calculateDT( TopologyGrid2D *_tg2d ) {
	
    _tg2d->CalculateDT( );
	gRidge = _tg2d->GetDTRidge();
	
	printf("--> Calculated 2D DT! \n");

}

/**
 * @function init
 */
bool init() {

	SIZE_X = 18; 
    SIZE_Y = 18;	


	SCREEN_WIDTH = TILE_WIDTH*SIZE_X;
	SCREEN_HEIGHT = TILE_HEIGHT*SIZE_Y;

	MOUSE_MODE = -1;
	mStart.x = -1; mStart.y = -1;
	mGoal.x = -1; mGoal.y = -1;

	//-- SDL Initialization
	if( SDL_Init( SDL_INIT_EVERYTHING ) == -1 ) {
		return false;
	}	
	
	//-- Setup the screen
	screen = SDL_SetVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, SDL_SWSURFACE );

	//-- If there was an error in setting up the screen
	if( screen == NULL ) {
		return false;
	}	

	//-- Set the window caption
	SDL_WM_SetCaption( "TG2D", NULL );

	//-- If everything initialized fine
	return true;
}

/**
 * @function createBorderingObstacles
 */
void createBorderingObstacles( Grid2D *_g ) {

	Pos p; 

	for( int i = 0; i < SIZE_X; ++i ) {

		p.x = i; p.y = 0;
		_g->SetState( p, true );

		p.x = i; p.y = SIZE_Y - 1; 
		_g->SetState( p, true );
	} 	

	for( int j = 0; j < SIZE_Y; ++j ) {

		p.x = 0; p.y = j;
		_g->SetState( p, true );

		p.x = SIZE_X - 1; p.y = j; 
		_g->SetState( p, true );
	} 	

}

/**
 * @function printPaths
 */
void printPaths() {

	for( int i = 0; i < gPaths.size(); ++i ) {
		for( int j = 0; j < gPaths[i].size(); ++j ) {
			printf(" [%d] (%d %d) \n", i, gPaths[i][j].x, gPaths[i][j].y );
		}
	}

}


