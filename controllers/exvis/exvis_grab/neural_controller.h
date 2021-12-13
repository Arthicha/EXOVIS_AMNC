#ifndef NEURAL_CONTROLLER_H
#define NEURAL_CONTROLLER_H

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <map>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "ann.h"
#include "so2cpg.h"
#include "neuron.h"
#include "synapse.h"
#include "pmn.h"
#include "dmp.h"
#include "rbf.h"
#include "exo_joint_module.h"
#include "exo_joint_module_parameters.h"
using namespace std;

#define JOINTNUM 6
#define PATH2RHIP "/rHip.csv"
#define PATH2RKNEE "/rKnee.csv"
#define PATH2RANKLE "/rAnkle.csv"
#define PATH2LHIP "/lHip.csv"
#define PATH2LKNEE "/lKnee.csv"
#define PATH2LANKLE "/lAnkle.csv"

#define R2D 57.2958


#define SOFTSTART 128
#define ASSIST 100

#define JOINTANGLETOPIC "/exvis/con/jointAngle"
#define JOINTFEEDBACKTOPIC "/exvis/jointAngle"


class EXO_DMP;

class neuralController : public ANN{
public:
	neuralController(int argc,char *argv[]);
	bool runController();

private:

	float timestep = 0;
	float mode = 0;
	float previous_mode = 0;
	float eta = 0;
	float previous_eta = 0;
	float type = 1;
	float previous_type = 1;
	
	// neural control module
	EXO_JOINT_MODULE * joint_module[JOINTNUM];

	// neural control parameter structure
	struct exoJointModuleParams module_params[JOINTNUM];


	// list of file names
	string listoffile[JOINTNUM] = {PATH2RHIP,PATH2RKNEE,PATH2RANKLE,PATH2LHIP,PATH2LKNEE,PATH2LANKLE};
	string jointnames[JOINTNUM] = {"rHip_module","rKnee_module","rAnkle_module","lHip_module","lKnee_module","lAnkle_module"};
	

	int upperLim[JOINTNUM] = {105,105,30,105,105,30};
	int lowerLim[JOINTNUM] = {-30,-5,-30,-30,-5,-30};
	float gain[JOINTNUM] = {1.0,1.0,1.0,1.0,1.0,1.0};//{1.0*0.75,2.5,1.0,1.0*0.75,2.5,1.0};
	float bias[JOINTNUM] = {0.0872665*0,0.0,0.3*0,0.0872665*0,0.0,0.3*0};

	// vectors
	vector<float> noise;

	// set reflesh rate
    ros::Rate* rosrate;
    ros::NodeHandle* node;

    // publisher
    ros::Publisher manJointPositionPub;
    ros::Publisher exoJointPositionPub;
    ros::Publisher debugMessagePub;
    ros::Publisher exoModePub;
    ros::Publisher exoSpeedPub;
    ros::Publisher exoAssistPub;
    ros::Publisher cpgPub;
    ros::Publisher errorPub;
    ros::Publisher gradientPub;
    ros::Publisher impedanceControlPub;


    // subscriber
    ros::Subscriber manJointFeedbackSub;
    ros::Subscriber exoJointFeedbackSub;

    // message
    std_msgs::Float32MultiArray manJointPosition;
    std_msgs::Int32MultiArray exoJointPosition;
    std_msgs::Float32MultiArray manJointFeedback;
    std_msgs::Int32MultiArray exoJointFeedback;
    std_msgs::Float32MultiArray debugMessage;
    std_msgs::Float32MultiArray cpgMsg;
    std_msgs::Float32MultiArray errorMsg;
    std_msgs::Float32MultiArray gradientMsg;
    std_msgs::Float32MultiArray impedanceControlMsg;

    std_msgs::Int32 exoMode;
    std_msgs::Int32 exoSpeed;
    std_msgs::Int32 exoAssist;

    // methods
	void manJointFeedbackCB(const std_msgs::Float32MultiArray::ConstPtr& array);
	void exoJointFeedbackCB(const std_msgs::Int32MultiArray::ConstPtr& array);

	int clip(float x, int mini,int maxi);

	void setGradFeedback(int i, int j, float gain);
		

}; 
#endif // NEURAL_CONTROLLER_H
