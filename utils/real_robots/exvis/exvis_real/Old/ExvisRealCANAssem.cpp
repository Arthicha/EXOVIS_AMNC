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

#include "ExvisRealCANAssem.h"

//***************************************************************************

ExvisRealCANAssem::ExvisRealCANAssem(int argc,char* argv[]) {

	realRos = new ExvisRealCANROS(argc, argv);

	comCAN = new CommuCAN();

    if(ros::ok()) {
    	cout << "Initialize ExvisRealCANAssem" << endl;
    }

}


bool ExvisRealCANAssem::runAssem() {
	if(ros::ok()){
		cout << "********************************************************" << endl;
		cout << "ExvisRealCANAssem is running" << endl;
		cout << nodeCounter << endl;



		cout << "--------------- Send control command ---------------" << endl;
		// Outside to ROS to CAN
		comCAN->type = realRos->type;
		comCAN->speed = realRos->speed;
		comCAN->dataPts = realRos->dataPts;
		comCAN->assist = realRos->assist;
		cout << "type: " << comCAN->type << " "
			<< "speed: " << comCAN->speed << " "
			<< "dataPts: " << comCAN->dataPts << " "
			<< "assist: " << comCAN->assist
			<< endl;

		// ROS to CAN -- Put vector into vector: jointAngleCon
		if (comCAN->iJointAngleCon == (comCAN->dataPts - 1)){
			comCAN->iJointAngleCon = 0;
		}
		comCAN->jointAngleCon[comCAN->iJointAngleCon] = realRos->jointAngleCon;
		comCAN->iJointAngleCon++;
		printf("  - W %s DATA:%02x %02x %02x %02x %02x %02x \n",
			"JointAngle",
			(int)realRos->jointAngleCon[0], (int)realRos->jointAngleCon[1], (int)realRos->jointAngleCon[2],
			(int)realRos->jointAngleCon[3], (int)realRos->jointAngleCon[4], (int)realRos->jointAngleCon[5]);

		// ROS to CAN -- Put vector into vector: stiffnessCon
		if (comCAN->iStiffnessCon == (comCAN->dataPts - 1)){
			comCAN->iStiffnessCon = 0;
		}
		comCAN->stiffnessCon[comCAN->iStiffnessCon] = realRos->stiffnessCon;
		comCAN->iStiffnessCon++;
		printf("  - W %s DATA:%02x %02x %02x %02x %02x %02x \n",
			"Stiffness",
			(int)realRos->stiffnessCon[0], (int)realRos->stiffnessCon[1], (int)realRos->stiffnessCon[2],
			(int)realRos->stiffnessCon[3], (int)realRos->stiffnessCon[4], (int)realRos->stiffnessCon[5]);

		// ROS to CAN -- Put vector into vector: torqueCon
		if (comCAN->iTorqueCon == (comCAN->dataPts - 1)){
			comCAN->iTorqueCon = 0;
		}
		comCAN->torqueCon[comCAN->iTorqueCon] = realRos->torqueCon;
		comCAN->iTorqueCon++;
		printf("  - W %s DATA:%02x %02x %02x %02x %02x %02x \n",
			"Torque",
			(int)realRos->torqueCon[0], (int)realRos->torqueCon[1], (int)realRos->torqueCon[2],
			(int)realRos->torqueCon[3], (int)realRos->torqueCon[4], (int)realRos->torqueCon[5]);


		// + Pub/Sub area --------------------------------
		cout << "--------------- Published ---------------" << endl;
		clock_gettime(CLOCK_REALTIME, &tNode);
		cout << "tPublished: " << to_string(tNode.tv_sec + tNode.tv_nsec/1e9) << endl;


		// CAN to ROS to outside
		realRos->sendTimeStampCAN(comCAN->timeStampCAN);
		realRos->sendJointAngle(comCAN->jointAngleData);
		realRos->sendJointTorque(comCAN->jointTorqueData);
		realRos->sendFootSwitch(comCAN->footSwitchData);
		realRos->sendMotorTorque(comCAN->motorTorqueData);

		realRos->sendOneCycleWrite(comCAN->oneCycleWrite);

		// - Pub/Sub area --------------------------------



		// + Node spin to have sub value --------------------------------
		clock_gettime(CLOCK_REALTIME, &tNode);
		cout << "tSubscribed: " << to_string(tNode.tv_sec + tNode.tv_nsec/1e9) << endl;
		realRos->rosSpinOnce();
		// - Node spin to have sub value --------------------------------
		nodeCounter++;
		cout << "********************************************************" << endl;
		return true;


	} else {
		cout << "Shutting down the node" << endl;
		return false;
	}
}

ExvisRealCANAssem::~ExvisRealCANAssem() {

}
