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

#include "ExvisRealCANROS.h"

//***************************************************************************

ExvisRealCANROS::ExvisRealCANROS(int argc, char **argv) {
    // Create a ROS nodes
    int _argc = 0;
    char** _argv = NULL;
    ros::init(_argc,_argv,"ExvisRealCANROS");

    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");

    ros::NodeHandle node("~");
    ROS_INFO("ExvisRealCANROS just started!");

    // Initialize Subscribers
    typeConSub = node.subscribe("/exvis/con/type", 1, &ExvisRealCANROS::typeCon_CB, this);
    speedConSub = node.subscribe("/exvis/con/speed", 1, &ExvisRealCANROS::speedCon_CB, this);
    dataPtsConSub = node.subscribe("/exvis/con/dataPts", 1, &ExvisRealCANROS::dataPtsCon_CB, this);
    assistConSub = node.subscribe("/exvis/con/assist", 1, &ExvisRealCANROS::assistCon_CB, this);
    jointAngleConSub = node.subscribe("/exvis/con/jointAngle", 1, &ExvisRealCANROS::jointAngleCon_CB, this);


    // Initialize Publishers
    timeStampCANPub = node.advertise<std_msgs::String>("/exvis/timeStampCAN",1);
    jointAnglePub = node.advertise<std_msgs::Int32MultiArray>("/exvis/jointAngle",1);
    jointTorquePub = node.advertise<std_msgs::Int32MultiArray>("/exvis/jointTorque",1);
    footSwitchPub = node.advertise<std_msgs::Int32MultiArray>("/exvis/footSwitch",1);
    motorTorquePub = node.advertise<std_msgs::Int32MultiArray>("/exvis/motorTorque",1);
    oneCycleWritePub = node.advertise<std_msgs::Bool>("/exvis/oneCycleWrite",1);

    // Set Rate
    //rate = new ros::Rate(17*4); // 60hz
    rate = new ros::Rate(100);
//    rate = new ros::Rate(0.22);
//    rate = new ros::Rate(11.1);
//    rate = new ros::Rate(10);
}


// **********************************************************
// Subscriber callback
// **********************************************************
void ExvisRealCANROS::typeCon_CB(const std_msgs::Int32& _typeCon) {

    // Get the data from Topic into main node variable
	type = _typeCon.data;
	string typeString;
	switch (type) {
		case 1 :	typeString = "Position control";
					break;
		case 2 :	typeString = "Stiffness control";
					break;
		case 3 :	typeString = "Torque control";
					break;
		case 4 :	typeString = "Motor disable";
					break;
		case 5 :	typeString = "Motor stop";
					break;
	}
	cout << "type: " << typeString << endl;
}

void ExvisRealCANROS::speedCon_CB(const std_msgs::Int32& _speedCon) {

    // Get the data from Topic into main node variable
	speed = _speedCon.data;
	cout << "speed: " << speed << endl;
}

void ExvisRealCANROS::dataPtsCon_CB(const std_msgs::Int32& _dataPtsCon) {

    // Get the data from Topic into main node variable
	dataPts = _dataPtsCon.data;
	cout << "dataPts: " << dataPts << endl;
}

void ExvisRealCANROS::assistCon_CB(const std_msgs::Int32& _assistCon) {

    // Get the data from Topic into main node variable
	assist = _assistCon.data;
	cout << "assist: " << assist << endl;
}

void ExvisRealCANROS::jointAngleCon_CB(const std_msgs::Int32MultiArray& _jointAngleCon) {

    // Get the data from Topic into main node variable
	jointAngleCon = _jointAngleCon.data;
	cout << "jointAngle: " << jointAngleCon[0] << " " << jointAngleCon[1] << " " << jointAngleCon[2] << " " << jointAngleCon[3] << " " << jointAngleCon[4] << " " << jointAngleCon[5] << endl;

}

void ExvisRealCANROS::stiffnessCon_CB(const std_msgs::Int32MultiArray& _stiffnessCon) {

    // Get the data from Topic into main node variable
	stiffnessCon = _stiffnessCon.data;
	cout << "stiffness: " << stiffnessCon[0] << " " << stiffnessCon[1] << " " << stiffnessCon[2] << " " << stiffnessCon[3] << " " << stiffnessCon[4] << " " << stiffnessCon[5] << endl;

}

void ExvisRealCANROS::torqueCon_CB(const std_msgs::Int32MultiArray& _torqueCon) {

    // Get the data from Topic into main node variable
	torqueCon = _torqueCon.data;
	cout << "torque: " << torqueCon[0] << " " << torqueCon[1] << " " << torqueCon[2] << " " << torqueCon[3] << " " << torqueCon[4] << " " << torqueCon[5] << endl;

}

// **********************************************************
// Publisher send function
// **********************************************************

// sendTimeStampCAN ---------------------
void ExvisRealCANROS::sendTimeStampCAN(timespec timeStampCANs) {
    std_msgs::String sText;
    sText.data = to_string(timeStampCANs.tv_sec + timeStampCANs.tv_nsec/1e9);
    timeStampCANPub.publish(sText);
    cout << "timeStampCAN: " << sText.data << endl;
    cout << "... timeStampCAN Published !" << endl;
}


// sendJointAngle ---------------------
void ExvisRealCANROS::sendJointAngle(vector<int> jointAngles) {
    std_msgs::Int32MultiArray array;
    array.data.clear();

    for (int vJointAngle : jointAngles) {
    	// Convert value to have sign
    	if ((vJointAngle >= 128) && (vJointAngle <= 255)){
    		vJointAngle = vJointAngle - 256;
    	}
        array.data.push_back(vJointAngle);
    }

    jointAnglePub.publish(array);
    cout << "- R JointAngle " << array.data[0] << " " << array.data[1] << " " << array.data[2] << " " << array.data[3] << " " << array.data[4] << " " << array.data[5] << endl;
    cout << "... jointAngle Published !" << endl;
}

// sendJointTorque -------------------
void ExvisRealCANROS::sendJointTorque(vector<int> jointTorques) {
    std_msgs::Int32MultiArray array;
    array.data.clear();

    for (int vJointTorque : jointTorques) {
    	// Convvert value to have sign
    	if ((vJointTorque >= 128) && (vJointTorque <= 255)){
    		vJointTorque = vJointTorque - 256;
    	}
        array.data.push_back(vJointTorque);
    }

    jointTorquePub.publish(array);
    cout << "... jointTorque Published !" << endl;
}

// sendFootSwitch -----------------
void ExvisRealCANROS::sendFootSwitch(vector<int> footSwitches) {
    std_msgs::Int32MultiArray array;
    array.data.clear();

    for (int vFootSwitch : footSwitches) {
        array.data.push_back(vFootSwitch);
    }

    footSwitchPub.publish(array);
    cout << "... footSwitch Published !" << endl;
}


// sendMotorTorque ----------------
void ExvisRealCANROS::sendMotorTorque(vector<int> motorTorques) {
    std_msgs::Int32MultiArray array;
    array.data.clear();

    for (int vMotorTorque : motorTorques) {
        array.data.push_back(vMotorTorque);
    }

    motorTorquePub.publish(array);
    cout << "... motorTorque Published !" << endl;
}

void ExvisRealCANROS::sendOneCycleWrite(bool oneCycleWrites) {
	std_msgs::Bool bValue;
	bValue.data = oneCycleWrites;
	oneCycleWritePub.publish(bValue);
	cout << "oneCycleWrite: " << (bValue.data ? "true": "false") << endl;
	cout << "... oneCycleWrite Published" << endl;
}

// **********************************************************
// rosSpinOnce
// **********************************************************
void ExvisRealCANROS::rosSpinOnce(){

	cout << "ExvisRealCANROS node is spinning" << endl;

	ros::spinOnce();
	bool rateMet = rate->sleep();

    if(!rateMet)
    {
        ROS_ERROR("Sleep rate not met");
    }

//    cout << "Sleep finish !!!" << endl;
}

ExvisRealCANROS::~ExvisRealCANROS() {
    ROS_INFO("ExvisRealCANROS just terminated!");
    ros::shutdown();
}
