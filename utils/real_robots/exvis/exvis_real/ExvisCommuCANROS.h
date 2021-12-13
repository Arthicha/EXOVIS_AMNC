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

#ifndef EXVISCOMMUCANROS_H
#define EXVISCOMMUCANROS_H

#include <stdio.h> // standard input / output functions
#include <unistd.h> // standard function definitions
#include <stdlib.h> // variable types, macros, functions
#include <string.h> // string function definitions
#include <iostream> // C++ to use cout
#include <fstream> // To read/write on file
#include <vector>
#include <math.h>

using namespace std;

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>


//***************************************************************************

class ExvisCommuCANROS {
	public:
		// Public Global Variables

		// Write data ------------------------
		// Type of control
		// 1. Position control
		// 2. Stiffness control
		// 3. Torque control
		// 4. Motor disable
		// 5. Motor stop
		int type = 4; // 4 Default is Motor disable
		int speed = 1; // Default speed
		int dataPts = 51; // Default for number of data/walking cycle
		int assist = 60; // Default percentage of assistance
		vector<int> jointAngleCon = {0, 0, 0, 0, 0, 0};
		vector<int> stiffnessCon = {0, 0, 0, 0, 0, 0};
		vector<int> torqueCon = {0, 0, 0, 0, 0, 0};


		// Public Methods
		ExvisCommuCANROS(int argc, char *argv[]);
		~ExvisCommuCANROS();


		void sendTimeStampCAN(timespec timeStampCANs);
		void sendJointAngle(vector<int> jointAngles);
		void sendJointTorque(vector<int> jointTorques);
		void sendFootSwitch(vector<int> footSwitches);
		void sendMotorTorque(vector<int> motorTorques);

		void rosSpinOnce();



	private:
		// Subscribers
		ros::Subscriber typeConSub;
		ros::Subscriber speedConSub;
		ros::Subscriber dataPtsConSub;
		ros::Subscriber assistConSub;
		ros::Subscriber jointAngleConSub;
		ros::Subscriber stiffnessConSub;
		ros::Subscriber torqueConSub;


		// Publishers
		ros::Publisher timeStampCANPub;
		ros::Publisher jointAnglePub;
		ros::Publisher jointTorquePub;
		ros::Publisher footSwitchPub;
		ros::Publisher motorTorquePub;

		// Private Global Variables
		ros::Rate* rate;

		// Private Methods
		void typeCon_CB(const std_msgs::Int32&);
		void speedCon_CB(const std_msgs::Int32&);
		void dataPtsCon_CB(const std_msgs::Int32&);
		void assistCon_CB(const std_msgs::Int32&);
		void jointAngleCon_CB(const std_msgs::Int32MultiArray&);
		void stiffnessCon_CB(const std_msgs::Int32MultiArray&);
		void torqueCon_CB(const std_msgs::Int32MultiArray&);

};


#endif //EXVISCOMMUCANROS_H
