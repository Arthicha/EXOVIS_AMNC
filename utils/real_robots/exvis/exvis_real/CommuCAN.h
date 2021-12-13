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
#ifndef COMMUCAN_H
#define COMMUCAN_H

#include <stdio.h> // standard input / output functions
#include <unistd.h> // standard function definitions
#include <stdlib.h> // variable types, macros, functions
#include <string.h> // string function definitions
#include <iostream> // C++ to use cout
#include <vector>
#include <asm/types.h>
#include <sys/time.h>
#include <math.h>


using namespace std;

#include "PCANBasic.h"


#define PCAN_DEVICE		PCAN_USBBUS1
//***************************************************************************

class CommuCAN {
	public:
		CommuCAN();
		~CommuCAN();


		// Write data ------------------------------------------
		int updateRate = 0;




		TPCANMsg msgDisableCAN(); // CAN message to disable CAN communication
		TPCANMsg msgMinAngleCAN(); // Set min angle
		TPCANMsg msgMaxAngleCAN(); // Set max angle
		TPCANMsg msgPositionConCAN(); // Type of contro: Position control
		TPCANMsg msgTorqueConCAN(); // Type of contro: Torque control
		TPCANMsg msgMotorDisableCAN(); // Type of contro: Motor disable (Passive)
		TPCANMsg msgMotorStopCAN(); // Type of contro: Motor stop
		TPCANMsg msgPerAssistCAN(int per); // Percentage of assistance
		TPCANMsg msgStepPositionCAN(vector<int> position);
		TPCANMsg msgStepTorqueCAN(vector<int> torque);

		int writeCAN (TPCANMsg message, int writeCANID);
		double speedToUpdateRate(int speed, int dataPts);



		// Read data ------------------------------------------
		// Variables for outside
		timespec timeStampCAN;
		int canIDData;
		vector<int> jointAngleData = {0, 0, 0, 0, 0, 0};
		vector<int> jointTorqueData = {0, 0, 0, 0, 0, 0};
		vector<int> footSwitchData = {0, 0, 0, 0, 0, 0};
		vector<int> motorTorqueData = {0, 0, 0, 0, 0, 0};

		// CAN method
		int readCAN(int readCANID); // It will call readMessage().

	private:

		TPCANStatus statusCAN;
		TPCANMsg messageCAN;

		// Write data ------------------------------------------
		int writeMessage(TPCANMsg message, timespec * start, bool waitkey, int writeCANID);

		// Read data ------------------------------------------
		// CAN variables

		timespec startCAN;
		TPCANMsg readMessageCAN;

		int readMessage(TPCANMsg* curMessage, timespec * start, bool waitkey, int readCANID);




};



#endif // COMMUCAN_H
