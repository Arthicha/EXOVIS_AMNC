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

#include "ExvisCommuCANROS.h"

//***************************************************************************

ExvisCommuCANROS::ExvisCommuCANROS(int argc, char **argv) {
    // Create a ROS nodes
    int _argc = 0;
    char** _argv = NULL;
    ros::init(_argc,_argv,"ExvisCommuCANAssem");

    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");

    ros::NodeHandle node("~");
    ROS_INFO("ExvisRealCANROS just started!");

    // Initialize Subscribers

    // Initialize Publishers
    nodeCounterPub = node.advertise<std_msgs::Int32>("/exvis/commucan/nodeCounter",1);

    // Set Rate
    //rate = new ros::Rate(17*4); // 60hz
//    rate = new ros::Rate(100);
//    rate = new ros::Rate(0.22);
//    rate = new ros::Rate(11.1);
//    rate = new ros::Rate(10);
}


// **********************************************************
// Subscriber callback
// **********************************************************


// **********************************************************
// Publisher send function
// **********************************************************
void ExvisCommuCANROS::sendNodeCounter(int nodeCounters) {
	std_msgs::Int32 nodeCounterP;
	nodeCounterP.data = nodeCounters;
	nodeCounterPub.publish(nodeCounterP);
	cout << "... nodeCounter Published !" << endl;
}


// **********************************************************
// rosSpinOnce
// **********************************************************
void ExvisCommuCANROS::rosSpinOnce(){

	cout << "ExvisCommuCANROS node is spinning" << endl;

	ros::spinOnce();
	bool rateMet = rate->sleep();

    if(!rateMet)
    {
        ROS_ERROR("Sleep rate not met");
    }

//    cout << "Sleep finish !!!" << endl;
}

ExvisCommuCANROS::~ExvisCommuCANROS() {
    ROS_INFO("ExvisCommuCANROS just terminated!");
    ros::shutdown();
}
