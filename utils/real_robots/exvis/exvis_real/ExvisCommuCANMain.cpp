/***************************************************************************
 *   Copyright (C) 2019 by BRAIN, VISTEC	                               *
 *                                    									   *
 *    chaicharn.a_s17@vistec.ac.th                                         *
 *    																	   *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 ***************************************************************************/

//***************************************************************************
// LIBRARY
//***************************************************************************
// Standard  --------------------------------------------------
#include <stdio.h> // standard input / output functions
#include <unistd.h> // standard function definitions
#include <stdlib.h> // variable types, macros, functions
#include <string.h> // string function definitions
#include <iostream> // C++ to use cout
#include <vector>
#include <cmath>
#include <time.h>   // time calls

using namespace std; // To use cout without std

// Controller --------------------------------------------------
#include "ExvisCommuCANAssem.h"

//***************************************************************************

int main(int argc,char* argv[]){

    printf("\n****************************************\n");
	printf("\nSTARTING EXO CAN\n");
	printf("\nwritten by Akkawutvanich C.\n");
	printf("\n****************************************\n");

    
	ExvisCommuCANAssem robotCommuCAN(argc,argv);

    while(robotCommuCAN.runAssem()){}

    return 0;
}


