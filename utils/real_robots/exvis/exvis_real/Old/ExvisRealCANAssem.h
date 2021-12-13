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

#ifndef EXVISREALCANASSEM_H
#define EXVISREALCANASSEM_H

#include <stdio.h> // standard input / output functions
#include <unistd.h> // standard function definitions
#include <stdlib.h> // variable types, macros, functions
#include <string.h> // string function definitions
#include <iostream> // C++ to use cout
#include <fstream> // To read/write on file
#include <vector>
#include <sys/time.h>

using namespace std;

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>

#include "ExvisRealCANROS.h"
#include "CommuCAN.h"

//***************************************************************************

class ExvisRealCANROS;

class ExvisRealCANAssem {
	public:
		ExvisRealCANAssem(int argc,char* argv[]);
		~ExvisRealCANAssem();
		bool runAssem();

	private:
		ExvisRealCANROS * realRos;
		CommuCAN * comCAN;

		int nodeCounter = 0;
		timespec tNode;

};



#endif // EXVISREALCANASSEM_H
