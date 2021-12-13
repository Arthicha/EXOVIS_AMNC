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
#include "CommuCAN.h"

//***************************************************************************

CommuCAN::CommuCAN(){

	cout << "Initialize CommuCAN object" << endl;
	statusCAN = CAN_Initialize(PCAN_DEVICE, PCAN_BAUD_1M, 0, 0, 0);
	printf("Initialize CAN: %x\n", (int)statusCAN);

	messageCAN.LEN = 6;
	messageCAN.MSGTYPE = PCAN_MESSAGE_STANDARD;

	// Set min angle --------------
	cout << "Set min angles" << endl;
	writeCAN(msgMinAngleCAN(), 75);

	// Set max angle --------------
	cout << "Set max angles" << endl;
	writeCAN(msgMaxAngleCAN(), 80);

	// Set type of control: Motor disable (Passive) --------------
	cout << "Start with motor disable (Passive)" << endl;
	writeCAN(msgMotorDisableCAN(), 71);

}


//***************************************
//* FUNCTION to set frequently use CAN message
//***************************************
// Type of control: Position control -----------------------------------------------
TPCANMsg CommuCAN::msgPositionConCAN(){

	TPCANMsg message;
	message.ID = 0x47; // 71
	message.LEN = 6;
	message.MSGTYPE = PCAN_MESSAGE_STANDARD;
	message.DATA[0]= 1;	message.DATA[1]= 1;	message.DATA[2]= 1;
	message.DATA[3]= 1;	message.DATA[4]= 1;	message.DATA[5]= 1;
	return message;
}

// Type of control: Motor disable (Passive) -----------------------------------------------
TPCANMsg CommuCAN::msgMotorDisableCAN(){

	TPCANMsg message;
	message.ID = 0x47; // 71
	message.LEN = 6;
	message.MSGTYPE = PCAN_MESSAGE_STANDARD;
	message.DATA[0]= 4;	message.DATA[1]= 4;	message.DATA[2]= 4;
	message.DATA[3]= 4;	message.DATA[4]= 4;	message.DATA[5]= 4;
	return message;
}

// Type of control: Motor stop -----------------------------------------------
TPCANMsg CommuCAN::msgMotorStopCAN(){

	TPCANMsg message;
	message.ID = 0x47; // 71
	message.LEN = 6;
	message.MSGTYPE = PCAN_MESSAGE_STANDARD;
	message.DATA[0]= 5;	message.DATA[1]= 5;	message.DATA[2]= 5;
	message.DATA[3]= 5;	message.DATA[4]= 5;	message.DATA[5]= 5;
	return message;
}

// Position set point -----------------------------------------------
TPCANMsg CommuCAN::msgStepPositionCAN(vector<int> position){

	TPCANMsg message;
	message.ID = 0x48; // 72
	message.LEN = 6;
	message.MSGTYPE = PCAN_MESSAGE_STANDARD;
	message.DATA[0]= position[0];	message.DATA[1]= position[1];	message.DATA[2]= position[2];
	message.DATA[3]= position[3];	message.DATA[4]= position[4];	message.DATA[5]= position[5];
	return message;
}


// Set min angle -----------------------------------------------
TPCANMsg CommuCAN::msgMinAngleCAN(){

	TPCANMsg message;
	message.ID = 0x4B; // 75
	message.LEN = 6;
	message.MSGTYPE = PCAN_MESSAGE_STANDARD;
	message.DATA[0]= -25;	message.DATA[1]= 0;	message.DATA[2]= -25;
	message.DATA[3]= -25;	message.DATA[4]= 0;	message.DATA[5]= -25;
	return message;
}

// Percentage of assistance -----------------------------------------------
TPCANMsg CommuCAN::msgPerAssistCAN(int per){

	TPCANMsg message;
	message.ID = 0x4C; // 76
	message.LEN = 6;
	message.MSGTYPE = PCAN_MESSAGE_STANDARD;
	message.DATA[0]= per;	message.DATA[1]= per;	message.DATA[2]= per;
	message.DATA[3]= per;	message.DATA[4]= per;	message.DATA[5]= per;
	return message;
}

// Set max angle -----------------------------------------------
TPCANMsg CommuCAN::msgMaxAngleCAN(){

	TPCANMsg message;
	message.ID = 0x50; // 80
	message.LEN = 6;
	message.MSGTYPE = PCAN_MESSAGE_STANDARD;
	message.DATA[0]= 100;	message.DATA[1]= 100;	message.DATA[2]= 25;
	message.DATA[3]= 100;	message.DATA[4]= 100;	message.DATA[5]= 25;
	return message;
}

// Disable CAN data -----------------------------------------------
TPCANMsg CommuCAN::msgDisableCAN(){

	TPCANMsg message;
	message.ID = 0x55; // 85
	message.LEN = 6;
	message.MSGTYPE = PCAN_MESSAGE_STANDARD;
	message.DATA[0]= 0;	message.DATA[1]= 0;	message.DATA[2]= 0;
	message.DATA[3]= 0;	message.DATA[4]= 0;	message.DATA[5]= 0;
	return message;
}



//***************************************
//* FUNCTION
//***************************************
// speed to updateRate -----------------------------------------------
double CommuCAN::speedToUpdateRate(int speed, int dataPts){
	double updateRate = 0.0; // sec: update rate of data
	double walkCycle = 0.0; // sec: how much time per walking cycle

	walkCycle = (-0.3)*speed + 4.8; // From EXO-H3 manual, total time for 1 walking step in sec

	updateRate = walkCycle/dataPts; //

	cout << "updateRate: " << updateRate << endl;

	return updateRate;
}


// ------------------------------------------------------------
// Write CAN data
// ------------------------------------------------------------

int CommuCAN::writeCAN(TPCANMsg message, int writeCANID){

	clock_gettime(CLOCK_REALTIME, &startCAN);
	writeMessage(message, &startCAN, false, writeCANID);

	return 0;
}


int CommuCAN::writeMessage(TPCANMsg message, timespec * start, bool waitkey, int writeCANID){

	TPCANStatus status;
	char stringKey[5];
	timespec end;

	clock_gettime(CLOCK_REALTIME, &end);

	double dif = (end.tv_sec + end.tv_nsec / 1e9)-((*start).tv_sec + (*start).tv_nsec / 1e9);
	*start = end;

	string writeCANIDString;
	switch (writeCANID) {
		case 71 :	writeCANIDString = "Type set";
					break;
		case 72 :	writeCANIDString = "Pos. set";
					break;
		case 73 :	writeCANIDString = "Trq. set";
					break;
		case 74 :	writeCANIDString = "Stiff. set";
					break;
		case 75 :	writeCANIDString = "Min. angles set";
					break;
		case 76 :	writeCANIDString = "Assist. set";
					break;
		case 80 :	writeCANIDString = "Max. angles set";
					break;
	}

	printf("  - W %s ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x   Elapsed time: %f\n",
		writeCANIDString.c_str(), (int)message.ID, (int)message.LEN,
		(int)message.DATA[0], (int)message.DATA[1], (int)message.DATA[2],
		(int)message.DATA[3], (int)message.DATA[4], (int)message.DATA[5], dif);

	if ( (status = CAN_Write(PCAN_DEVICE,&message)) != PCAN_ERROR_OK ){
		printf("PCAN_ERROR");
		printf("Error 0x%x\n", (int)status);
		return -1;
	}

	if (waitkey) {
		scanf("%s", stringKey);
	}


	return 0;
}


// ------------------------------------------------------------
// Read CAN data
// ------------------------------------------------------------
int CommuCAN::readCAN(int readCANID){

//	cout << "|-----------------------I AM HERE in readCAN()----------------------|" << endl;
	clock_gettime(CLOCK_REALTIME, &startCAN);
	readMessage(&readMessageCAN, &startCAN, false, readCANID); // waitkey should be false all the time.

	switch (readCANID) {
		case 110 :	{
			cout << "save data 110" << endl;
			canIDData =  (int)readMessageCAN.ID;
			jointAngleData[0] = (int)readMessageCAN.DATA[0];
			jointAngleData[1] = (int)readMessageCAN.DATA[1];
			jointAngleData[2] = (int)readMessageCAN.DATA[2];
			jointAngleData[3] = (int)readMessageCAN.DATA[3];
			jointAngleData[4] = (int)readMessageCAN.DATA[4];
			jointAngleData[5] = (int)readMessageCAN.DATA[5];
			cout << "jointAngleData (DEC): " << jointAngleData[0] << " " << jointAngleData[1] << " " << jointAngleData[2] << " " << jointAngleData[3] << " " << jointAngleData[4] << " " << jointAngleData[5] << " " << endl;
			break;
		}
		case 120 :	{
			cout << "save data 120" << endl;
			canIDData =  (int)readMessageCAN.ID;
			jointTorqueData[0] = (int)readMessageCAN.DATA[0];
			jointTorqueData[1] = (int)readMessageCAN.DATA[1];
			jointTorqueData[2] = (int)readMessageCAN.DATA[2];
			jointTorqueData[3] = (int)readMessageCAN.DATA[3];
			jointTorqueData[4] = (int)readMessageCAN.DATA[4];
			jointTorqueData[5] = (int)readMessageCAN.DATA[5];
			break;
		}
		case 130 :	{
			cout << "save data 130" << endl;
			canIDData =  (int)readMessageCAN.ID;
			footSwitchData[0] = (int)readMessageCAN.DATA[0];
			footSwitchData[1] = (int)readMessageCAN.DATA[1];
			footSwitchData[2] = (int)readMessageCAN.DATA[2];
			footSwitchData[3] = (int)readMessageCAN.DATA[3];
			footSwitchData[4] = (int)readMessageCAN.DATA[4];
			footSwitchData[5] = (int)readMessageCAN.DATA[5];
			break;
		}
		case 140 :	{
			cout << "save data 140" << endl;
			canIDData =  (int)readMessageCAN.ID;
			motorTorqueData[0] = (int)readMessageCAN.DATA[0];
			motorTorqueData[1] = (int)readMessageCAN.DATA[1];
			motorTorqueData[2] = (int)readMessageCAN.DATA[2];
			motorTorqueData[3] = (int)readMessageCAN.DATA[3];
			motorTorqueData[4] = (int)readMessageCAN.DATA[4];
			motorTorqueData[5] = (int)readMessageCAN.DATA[5];
			break;
		}
	}

//	cout << "|----------------------I AM HERE out readCAN()----------------------|" << endl;
	return 0;

}


// Private: readMessage -----------------------------------------------
int CommuCAN::readMessage(TPCANMsg* curMessage, timespec * start, bool waitkey, int readCANID){

	cout << "---Start readMessage()---" << endl;

	TPCANStatus status;
	char stringKey[5];
	timespec end;

	clock_gettime(CLOCK_REALTIME, &end);

	double dif = (end.tv_sec + end.tv_nsec / 1e9)-((*start).tv_sec + (*start).tv_nsec / 1e9);
	*start = end;

	DWORD IDreceived = 0;
//	int counter = 0;

	string readCANIDString;
	switch (readCANID) {
		case 110 :	readCANIDString = "JointAngle";
					break;
		case 120 :	readCANIDString = "JointTorque";
					break;
		case 130 :	readCANIDString = "FootSwitch";
					break;
		case 140 :	readCANIDString = "MotorTorque";
					break;
	}


	// --------------- Check CAN status and retrieve data ---------------------
	while(1){


		while ((status = CAN_Read(PCAN_DEVICE, curMessage, NULL)) == PCAN_ERROR_QRCVEMPTY) {
			usleep(1000);
			cout << "PCAN_ERROR_QRCVEMPTY" << endl;
		}

		if (status != PCAN_ERROR_OK) {
			printf("PCAN_ERROR");
			printf("Error 0x%x\n", (int)status);
			return -1;
		}

		IDreceived = (*curMessage).ID;

//		// + Print everything -------------------------------------
//		printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x   Elapsed time: %f\n",
//			(int)(*curMessage).ID, (int)(*curMessage).LEN,
//			(int)(*curMessage).DATA[0], (int)(*curMessage).DATA[1],
//			(int)(*curMessage).DATA[2], (int)(*curMessage).DATA[3],
//			(int)(*curMessage).DATA[4], (int)(*curMessage).DATA[5], dif);

//		cout << "Before If: " << IDreceived << " " << readCANID << endl;
		// + Print data according to Message ID-------------------------------------
		if (IDreceived == readCANID){
//			cout << "In If: " << IDreceived << " " << readCANID << endl;
//			counter++;
			printf("  - R %s ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x   Elapsed time: %f\n",
				readCANIDString.c_str(), (int)(*curMessage).ID, (int)(*curMessage).LEN,
				(int)(*curMessage).DATA[0], (int)(*curMessage).DATA[1], (int)(*curMessage).DATA[2],
				(int)(*curMessage).DATA[3], (int)(*curMessage).DATA[4], (int)(*curMessage).DATA[5], dif);
			break;

		} else {
			cout << "IDs are not equal" << endl;
			cout << "IDreceived: " << IDreceived << " " << "readCANID: " << readCANID << endl;
		}

//		if (IDreceived == readCANID && counter > 200) break;

	}


	if (waitkey) {
		scanf("%s", stringKey);
	}
	cout << "---End readMessage()---" << endl;
	return 0;

}


//***************************************
//* Destructor
//***************************************
CommuCAN::~CommuCAN(){

}
