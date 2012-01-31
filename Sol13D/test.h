/**
 * @file test.h
 */

#ifndef TEST_H_
#define TEST_H_

#include <string>

#include "Grid3D.h"
//#include "TopologyGrid3D.h"
#include "Search.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

////////////// EXTERN VARIABLES //////////////////////
// Unless you want g++ spit redefinition errors :(  //
//////////////////////////////////////////////////////


//-- Global variables
extern std::vector< std::vector<Pos> > gPaths;
extern std::vector<Cell> gRidge;
extern int numPaths;
extern int countPath;
extern Pos mStart;
extern Pos mGoal;
extern Search *mSearch;


#endif /** _TEST_H_ */
