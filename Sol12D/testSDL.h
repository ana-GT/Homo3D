/**
 * @file testSDL.h
 */

#ifndef TEST_SDL_H_
#define TEST_SDL_H_

#include "SDL/SDL.h"
#include "SDL/SDL_image.h"
#include <string>

#include "Grid2D.h"
#include "TopologyGrid2D.h"
#include "Search.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

////////////// EXTERN VARIABLES //////////////////////
// Unless you want g++ spit redefinition errors :(  //
//////////////////////////////////////////////////////

//////////// SDL variables /////////////////////

//-- Surfaces
extern SDL_Surface *screen;
extern SDL_Surface *tiles;
extern SDL_Surface *paths;
extern SDL_Surface *v0;

//-- Attributes of the screen
extern int SCREEN_WIDTH;
extern int SCREEN_HEIGHT;
extern const int SCREEN_BPP;

//-- Tile constants
extern const int TILE_WIDTH;
extern const int TILE_HEIGHT;
extern const int TILE_SPRITES;

//-- Sprites
extern const int TILE_RED;
extern const int TILE_GREEN;
extern const int TILE_BLUE;
extern const int TILE_BLACK;
extern const int TILE_V0;

extern SDL_Rect clips[];
extern SDL_Rect clipsPaths[];

//-- Event structure
extern SDL_Event event;

//-- Global variables
extern int SIZE_X;
extern int SIZE_Y;
extern int MOUSE_MODE;
extern std::vector< std::vector<Pos> > gPaths;
extern std::vector<Cell> gRidge;
extern int numPaths;
extern int countPath;
extern Pos mStart;
extern Pos mGoal;
extern Search *mSearch;

//-- SDL Functions
bool init();
bool load_files();
void clean_up();
SDL_Surface *load_image( std::string filename );
void apply_surface( int x, int y, 
					SDL_Surface* source, 
					SDL_Surface *destination,
					SDL_Rect *clip );
Pos GetMousePos( int _x, int _y );
void display_options();
void clip_tiles();
void DrawScene( Grid2D *_g );
void DrawPath( int _index );
void DrawRidge();
void printPaths();


//-- Utility stuff
void calculateDT( TopologyGrid2D *_tg2d );
void createBorderingObstacles( Grid2D *_g );


#endif /** _TEST_SDL_H_ */
