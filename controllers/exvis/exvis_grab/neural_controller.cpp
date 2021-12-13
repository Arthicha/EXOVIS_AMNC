#include "neural_controller.h"

using namespace std;


neuralController::neuralController(int argc, char *argv[]) {

	/* ros initialization */
	int _argc = 0;
	char** _argv = nullptr;	

	// --------- initialize ROS ------------
	//******************************************  set up ROS  ********************************************//
    
    // create ros node
    std::string nodeName("ezo");
    ros::init(argc,argv,nodeName);
    this->node = new ros::NodeHandle("~");

    // check robot operating system
    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");
    ROS_INFO("ROS just started!");

    rosrate = new ros::Rate(30); // set ros-rate

    // create publisher/subscriber
    exoJointPositionPub =this->node->advertise<std_msgs::Int32MultiArray>(JOINTANGLETOPIC,1);
    manJointPositionPub =this->node->advertise<std_msgs::Float32MultiArray>("/manJointPosition",1);
    debugMessagePub =this->node->advertise<std_msgs::Float32MultiArray>("/debugMessage",1);
    exoModePub =this->node->advertise<std_msgs::Int32>("/exvis/con/type",1);
    exoSpeedPub =this->node->advertise<std_msgs::Int32>("/exvis/con/speed",1);
    exoAssistPub =this->node->advertise<std_msgs::Int32>("/exvis/con/assist",1);
    cpgPub =this->node->advertise<std_msgs::Float32MultiArray>("/exvis/con/cpg",1);
    errorPub =this->node->advertise<std_msgs::Float32MultiArray>("/exvis/con/error",1);
    gradientPub =this->node->advertise<std_msgs::Float32MultiArray>("/exvis/con/gradient",1);
    impedanceControlPub =this->node->advertise<std_msgs::Float32MultiArray>("/exvis/con/impedance",1);

    exoJointFeedbackSub =this->node->subscribe(JOINTFEEDBACKTOPIC,1,&neuralController::exoJointFeedbackCB,this);
    manJointFeedbackSub =this->node->subscribe("/manJointFeedback",1,&neuralController::manJointFeedbackCB,this);

    for(int i=0;i<JOINTNUM;i++){
    	exoJointPosition.data.push_back(0);
    	exoJointFeedback.data.push_back(0);
    	manJointPosition.data.push_back(0.0);
    	manJointFeedback.data.push_back(0.0);

    	errorMsg.data.push_back(0.0);
    	for(int j=0;j<2;j++)
    	{
    		cpgMsg.data.push_back(0.0);
    		gradientMsg.data.push_back(0.0);
    		impedanceControlMsg.data.push_back(0.0);
    	}	
    }

    for(int i=0;i<6;i++){debugMessage.data.push_back(0.0);}

    string path = "";
	this->node->getParam("/exvis/csvpath", path);
	this->node->getParam("/exvis/mode", this->mode);
	this->node->getParam("/exvis/eta", this->eta);
	this->node->getParam("/exvis/type", this->type);

    for(int i=0;i<JOINTNUM;i++)
    {
    	this->module_params[i].mode = this->mode;
    	this->module_params[i].name.assign(this->jointnames[i]);
    	this->module_params[i].filename.assign(path+this->listoffile[i]);
    	
    	this->joint_module[i] = new EXO_JOINT_MODULE(&module_params[i]);
    	addSubnet(this->joint_module[i]);
    }

    exoMode.data = this->type;exoSpeed.data = 10;exoAssist.data = ASSIST;
    exoModePub.publish(exoMode);
    exoSpeedPub.publish(exoSpeed);
    exoAssistPub.publish(exoAssist);
} // constructor


bool neuralController::runController() {

	float walk = 0.0;
	this->node->getParam("/exvis/walk", walk);
	this->node->getParam("/exvis/eta", this->eta);
	this->node->getParam("/exvis/type", this->type);

	if(walk > 0.0)
	{
		timestep += 1;
		
	}else{
		if(timestep > SOFTSTART){timestep = SOFTSTART;}
		if(timestep > 0){timestep -= 0.5;}
	}
	float temp_assist = ASSIST*timestep/SOFTSTART;
	if(temp_assist > ASSIST) {temp_assist = ASSIST;}
	exoAssist.data = (int) temp_assist;
	exoAssistPub.publish(exoAssist);
	
	if((timestep == SOFTSTART)||(this->eta != this->previous_eta))
	{	
		this->module_params[0].eta_cpg = this->eta;
		this->setGradFeedback(0,0,this->eta);
		this->setGradFeedback(3,0,this->eta);
		this->setGradFeedback(1,0,this->eta);
		this->setGradFeedback(4,0,this->eta);
		
		
		ROS_INFO_STREAM("phase adapt has been set" << endl);
		
		this->previous_eta = this->eta;
	}
	if(timestep <= SOFTSTART)
	{
		for(int i=0;i<JOINTNUM;i++){this->joint_module[i]->updateNewParams();}
	}

	if(this->type != this->previous_type)
	{
		if(timestep <= 0.0)
		{
			exoMode.data = this->type;
    		exoModePub.publish(exoMode);
    		this->previous_type = this->type;
		}
		
	}
	
	
	for(int i=0;i<JOINTNUM;i++)
	{
		this->joint_module[i]->setInput(2, ((((float)exoJointFeedback.data[i])/57.2958)-this->bias[i])/this->gain[i]);
	}

	for(int i=0;i<JOINTNUM;i++)
	{
		this->updateActivities();
	}
	for(int i=0;i<JOINTNUM;i++)
	{
		this->updateOutputs();
	}
		
	//cout << "*********************************************************" << endl;

	for(int i=0;i<JOINTNUM;i++)
	{
		exoJointPosition.data[i] = R2D*(this->joint_module[i]->getExoOutput()*this->gain[i])+this->bias[i];
		manJointPosition.data[i] = R2D*(this->joint_module[i]->getManOutput()*this->gain[i])+this->bias[i];
	}

	manJointPositionPub.publish(manJointPosition);
	exoJointPositionPub.publish(exoJointPosition);

	debugMessage.data[0] = this->joint_module[0]->getCPG(0,1);
	debugMessage.data[1] = this->joint_module[0]->getCPG(0,0);
	debugMessage.data[2] = this->joint_module[0]->getCPG(1,1);
	debugMessage.data[3] = this->joint_module[0]->getCPG(1,0);
	debugMessage.data[4] = ((float)exoJointFeedback.data[0])/57.2958;
	debugMessage.data[5] = exoJointPosition.data[0]/57.2958;
	debugMessagePub.publish(debugMessage);

	for(int i=0;i<JOINTNUM;i++)
	{
		cpgMsg.data[2*i] = this->joint_module[i]->getCPG(0,0);
		cpgMsg.data[2*i+1] = this->joint_module[i]->getCPG(0,1);
		errorMsg.data[i] = this->joint_module[i]->getDebug(0);
		gradientMsg.data[2*i] = (this->joint_module[i]->getGradNeuron(0)->getOutput());
		gradientMsg.data[2*i+1] = (this->joint_module[i]->getGradNeuron(1)->getOutput());
		impedanceControlMsg.data[2*i] = this->module_params[i].cp*(this->joint_module[i]->getDebug(1));
		impedanceControlMsg.data[2*i+1] = this->module_params[i].cd*(this->joint_module[i]->getDebug(1));
	}
	cpgPub.publish(cpgMsg);
	errorPub.publish(errorMsg);
	gradientPub.publish(gradientMsg);
	impedanceControlPub.publish(impedanceControlMsg);


	if (ros::ok())
	{
		ros::spinOnce();
		rosrate->sleep();
		return true;
	}else{
		return false;
	}
}//neuralController::runController()

void neuralController::exoJointFeedbackCB(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	int i = 0;
    for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        exoJointFeedback.data[i] = (*it);
        i++;
    }
    return;
} // neuralController::exoJointFeedbackCB

void neuralController::manJointFeedbackCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	int i = 0;
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        manJointFeedback.data[i] = *it;
        i++;
    }
    return;
} // neuralController::exoJointFeedbackCB

void neuralController::setGradFeedback(int i, int j, float gain)
{
	w(this->joint_module[i]->getCPGNeuron(0,0),this->joint_module[j]->getGradNeuron(0),gain);
	w(this->joint_module[i]->getCPGNeuron(0,1),this->joint_module[j]->getGradNeuron(1),gain);
}

int neuralController::clip(float x, int maxi,int mini)
{
	float xx = x*R2D;
	int y = (int) xx;
	if(y >= maxi){y = maxi;}
	if(y <= mini){y = mini;}
	return y;
}