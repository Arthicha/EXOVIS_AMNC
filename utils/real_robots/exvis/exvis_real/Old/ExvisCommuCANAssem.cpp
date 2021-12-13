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

#include "ExvisCommuCANAssem.h"

//***************************************************************************

ExvisCommuCANAssem::ExvisCommuCANAssem(int argc,char* argv[]) {

	realRos = new ExvisCommuCANROS(argc, argv);

	comCAN = new CommuCAN();

    if(ros::ok()) {
    	cout << "Initialize ExvisCommuCANAssem" << endl;
    }

}


bool ExvisCommuCANAssem::runAssem() {
	if(ros::ok()){
		cout << "********************************************************" << endl;
		cout << "ExvisCommuCANAssem is running" << endl;
		cout << nodeCounter << endl;

		// We cannot initiate only node without publisher, the node will not loop even if we have while(true)
		// However, we can omit rate and use computer speed instead.
		realRos->sendNodeCounter(nodeCounter);


		// --------------------------------------
		// Write data to CAN
		// --------------------------------------
		cout << "------------- Write data to CAN -------------" << endl;
		// Write percentage of assistance
		comCAN->writeCAN(comCAN->msgPerAssistCAN(comCAN->assist), 76);

		// Write type of control
		switch (comCAN->type) {

			// Position control
			case 1 :	{
				cout << "Type: Position control" << endl;

				comCAN->writeCAN(comCAN->msgPositionConCAN(), 71); // type of control

				comCAN->updateRate = comCAN->speedToUpdateRate(comCAN->speed, comCAN->dataPts);

				sleep(comCAN->updateRate); // Delay in sec according to updateRate

				if (comCAN->iWrite == (comCAN->dataPts - 1)){
					comCAN->iWrite = 0;
					comCAN->oneCycleWrite = true; // Will use to notify the world outside
				}
				comCAN->writeCAN(comCAN->msgStepPositionCAN(comCAN->jointAngleCon[comCAN->iWrite]), 72);
				comCAN->iWrite++;
				break;

			}

			// Stiffness control
			case 2 :	{
				cout << "Type: Stiffness control" << endl;
				// ??
				break;
			}

			// Torque control
			case 3 :	{
				cout << "Type: Torque control" << endl;
				// ??
				break;
			}

			// Motor disable
			case 4 :	{
				cout << "Type: Motor disable" << endl;
				comCAN->writeCAN(comCAN->msgMotorDisableCAN(), 71);
				break;
			}

			// Motor stop
			case 5 :	{
				cout << "Type: Motor stop" << endl;
				comCAN->writeCAN(comCAN->msgMotorStopCAN(), 71);
				break;
			}


		}


		// --------------------------------------
		// Read data from CAN
		// --------------------------------------
		cout << "------------- Read data from CAN -------------" << endl;

		// Timestamp when we read the data
		clock_gettime(CLOCK_REALTIME, &comCAN->timeStampCAN);
		cout << "timeStampCAN: " << to_string((comCAN->timeStampCAN).tv_sec + (comCAN->timeStampCAN).tv_nsec/1e9) << endl;

		// Message ID = 110 Joint angle -----
		comCAN->readCAN(110);

		// Message ID = 120 Joint torque -----
		comCAN->readCAN(120);

		// Message ID = 130 Foot switch -----
		comCAN->readCAN(130);

		// Message ID = 140 Motor torque -----
		comCAN->readCAN(140);




		// + Node spin to have sub value --------------------------------
		clock_gettime(CLOCK_REALTIME, &tNode);
		cout << "tSubscribed: " << to_string(tNode.tv_sec + tNode.tv_nsec/1e9) << endl;
//		realRos->rosSpinOnce();
		// - Node spin to have sub value --------------------------------
		nodeCounter++;
		cout << "********************************************************" << endl;
		return true;


	} else {
		cout << "Shutting down the node" << endl;
		return false;
	}
}

ExvisCommuCANAssem::~ExvisCommuCANAssem() {

}
