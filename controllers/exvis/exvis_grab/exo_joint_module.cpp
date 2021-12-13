#include "exo_joint_module.h"


using namespace std;


EXO_JOINT_MODULE::EXO_JOINT_MODULE(struct exoJointModuleParams* ptr){
	
	this->params_ptr = ptr;
	this->getTarget();

	this->init_ipn();
	for(int i=0;i<2;i++){this->init_cpg(i);}
	this->init_exo_rbf();
	this->init_man_rbf();
	this->addNoise();
	this->init_dmp();
	this->init_gradn();

	this->updateNewParams();
	

	if (this->params_ptr->phase_shift > 0){this->cpg_step(1,abs(this->params_ptr->phase_shift));}
	else if (this->params_ptr->phase_shift < 0){this->cpg_step(1,TARGETLENGTH-abs(this->params_ptr->phase_shift));}
	
}

// *****************************************************************************
// **                                                                         **
// **                           public method                                 **
// **                                                                         **
// *****************************************************************************

// -------------------          set/get methods       --------------------------

void EXO_JOINT_MODULE::setInput(int i, float value)
{
    setActivity(this->inputNeurons->getNeuron(i),value);
    setOutput(this->inputNeurons->getNeuron(i),value);
}


Neuron* EXO_JOINT_MODULE::getInputNeuron(int i)
{
	return this->inputNeurons->getNeuron(i);
}


Neuron* EXO_JOINT_MODULE::getExoOutputNeuron()
{
	return this->dmp->getOutputNeuron();
}

Neuron* EXO_JOINT_MODULE::getManOutputNeuron()
{
	return this->man_rbf->getNeuron(2);
}

Neuron* EXO_JOINT_MODULE::getCPGNeuron(int cpg_num, int neuron_num)
{
	return this->cpg[cpg_num]->getNeuron(neuron_num);
}

Neuron* EXO_JOINT_MODULE::getGradNeuron(int index)
{
	return this->gradn->getOutputNeuron(index);
}

float EXO_JOINT_MODULE::getExoOutput()
{
	//return this->exo_rbf->getOutput(2);
	return this->dmp->getDMPOutput();
}

float EXO_JOINT_MODULE::getManOutput()
{
	return this->man_rbf->getOutput(2);
}


float EXO_JOINT_MODULE::getCPG(int cpg_num, int neuron_num)
{
	return this->cpg[cpg_num]->getOutput(neuron_num);
}

float EXO_JOINT_MODULE::getDebug(int id)
{
	if (id == 0)
	{
		return this->dmp->getErrorNeuron()->getOutput();
	}else{
		return this->dmp->getImpedanceNeuron()->getOutput();
	}
}

// -------------------          update/handle methods       --------------------------


void EXO_JOINT_MODULE::step()
{
	this->setInput(3,this->goal);
	this->cpg_step(0,1);
	this->cpg_step(1,1);
	this->exo_rbf->step();
	this->man_rbf->step();
	this->dmp->step();
	this->gradn->step();
}

void EXO_JOINT_MODULE::updateActivities()
{
	this->cpg[0]->updateActivities();
	this->cpg[1]->updateActivities();
	this->exo_rbf->updateActivities();
	this->man_rbf->updateActivities();
	this->dmp->updateActivities();
	this->gradn->updateActivities();
}

void EXO_JOINT_MODULE::updateOutputs()
{
	this->cpg[0]->updateOutputs();
	this->cpg[1]->updateOutputs();
	this->exo_rbf->updateOutputs();
	this->man_rbf->updateOutputs();
	this->dmp->updateOutputs();
	this->gradn->updateOutputs();
}

void EXO_JOINT_MODULE::updateNewParams()
{
	// central pattern generator
	for(int i=0;i<2;i++){this->set_cpg_connection(i);}
	// integrator / dynamic movement primitive
	this->dmp->set_system_params(this->params_ptr->alpha,this->params_ptr->beta);
	this->dmp->set_impedance_params(this->params_ptr->kp,this->params_ptr->kd,this->params_ptr->cp,this->params_ptr->cd);
	this->dmp->set_time(this->params_ptr->dt);
	// gradient network
	this->gradn->setParams(this->params_ptr->phi_primary, this->params_ptr->gamma_primary);
	this->set_cpg_connection(0);
}

void EXO_JOINT_MODULE::updateMode()
{
	float error = 0.0;
	if (this->params_ptr->mode < 0.5){error = this->exo_rbf->fit(this->position_trajectory);}
	else{error = this->exo_rbf->fit(this->target_trajectory);}

	ROS_INFO_STREAM("average error of " << this->params_ptr->name << " : " << TRAINING_SKIP*error/TARGETLENGTH);
	this->dmp->updateMode(this->params_ptr->mode);
}

// *****************************************************************************
// **                                                                         **
// **                           private method                                **
// **                                                                         **
// *****************************************************************************

void EXO_JOINT_MODULE::cpg_step(int cpg_num, int timesteps)
{
	for(int i=0;i<timesteps;i++){this->cpg[cpg_num]->step();}
}

// initialize neural modules

void EXO_JOINT_MODULE::init_cpg(int module){

	this->cpg[module] = new PMN(2);
	addSubnet(this->cpg[module]);
	for(int i=0;i<2;i++){this->cpg[module]->setTransferFunction(i,this->cpg[module]->tanhFunction());}
	this->set_cpg_connection(module);
	this->reset_cpg_activity(module);
	this->cpg_step(module,10000);

}//neuralController::init_cpg()


void EXO_JOINT_MODULE::reset_cpg_activity(int module){
	this->cpg[module]->setOutput(1, 0.885); this->cpg[module]->setOutput(0, -0.59268);
	this->cpg[module]->setActivity(0, 0.885); this->cpg[module]->setActivity(1, -0.59268);
	for(int i=0;i<1000;i++)
	{
		this->cpg[module]->updateActivities();
		this->cpg[module]->updateOutputs();
	}
}//neuralController::init_cpg_activity()

void EXO_JOINT_MODULE::set_cpg_connection(int module){

	float newgamma = 0.0;
	float newphi = 0.0;
	if(module == 0){
		newgamma = this->params_ptr->gamma_primary;
		newphi = this->params_ptr->phi_primary;
	}else{
		newgamma = this->params_ptr->gamma_secondary;
		newphi = this->params_ptr->phi_secondary;
	}
	w(this->cpg[module]->getNeuron(0), this->cpg[module]->getNeuron(0), newgamma*cos(newphi));w(this->cpg[module]->getNeuron(0), this->cpg[module]->getNeuron(1), newgamma*sin(newphi));
	w(this->cpg[module]->getNeuron(1), this->cpg[module]->getNeuron(0), -newgamma*sin(newphi));w(this->cpg[module]->getNeuron(1), this->cpg[module]->getNeuron(1), newgamma*cos(newphi));

}//neuralController::init_cpg_connection()

void EXO_JOINT_MODULE::init_ipn()
{
	/*
	input neurons: 
		(1) input to cpg 1
		(2) input to cpg 2
		(3) theta_feedback
		(4) dmp goal
	*/
	this->inputNeurons = new PMN(4);
	addSubnet(this->inputNeurons);
	for(int i=0;i<4;i++){setTransferFunction(this->inputNeurons->getNeuron(i),identityFunction());}
}


void EXO_JOINT_MODULE::init_exo_rbf()
{
	this->exo_rbf = new RBF(this->params_ptr->kirnel);
	addSubnet(this->exo_rbf);

	w(this->exo_rbf->getNeuron(0),this->cpg[0]->getNeuron(0),1.0);
	w(this->exo_rbf->getNeuron(1),this->cpg[0]->getNeuron(1),1.0);

	this->exo_rbf->setSigma(this->params_ptr->sigma_primary);
	this->exo_rbf->setLearningRate(this->params_ptr->eta_rbf);
	this->exo_rbf->setTrainingIteration(TRAINING_ITERATION);
	this->exo_rbf->setTrainingPeriod(TARGETLENGTH);
	this->exo_rbf->setTrainingSkip(TRAINING_SKIP);
	this->exo_rbf->setPhi(TRAINING_PHI);
	this->exo_rbf->setGamma(TRAINING_GAMMA);
	this->exo_rbf->setPresteps(TRAINING_PRESTEP);

	float error = 0.0;
	if (this->params_ptr->mode == 0){error = this->exo_rbf->fit(this->position_trajectory);}
	else{error = this->exo_rbf->fit(this->target_trajectory);}

	ROS_INFO_STREAM("average error of " << this->params_ptr->name << " : " << TRAINING_SKIP*error/TARGETLENGTH);
}


void EXO_JOINT_MODULE::init_man_rbf()
{
	this->man_rbf = new RBF(this->params_ptr->kirnel);
	addSubnet(this->man_rbf);

	w(this->man_rbf->getNeuron(0),this->cpg[1]->getNeuron(0),1.0);
	w(this->man_rbf->getNeuron(1),this->cpg[1]->getNeuron(1),1.0);
	this->man_rbf->setSigma(this->params_ptr->sigma_secondary);
	this->man_rbf->setLearningRate(this->params_ptr->eta_rbf);
	this->man_rbf->setTrainingIteration(TRAINING_ITERATION);
	this->man_rbf->setTrainingPeriod(TARGETLENGTH);
	this->man_rbf->setTrainingSkip(TRAINING_SKIP);
	this->man_rbf->setPhi(TRAINING_PHI);
	this->man_rbf->setGamma(TRAINING_GAMMA);
	this->man_rbf->setPresteps(TRAINING_PRESTEP);

	float error = 0.0;
	if (this->params_ptr->mode == 0){error = this->man_rbf->fit(this->position_trajectory);}
	else{error = this->man_rbf->fit(this->target_trajectory);}

}

void EXO_JOINT_MODULE::init_dmp()
{
	this->dmp = new DMP(this->params_ptr->mode);
	addSubnet(this->dmp);

	w(this->dmp->getInputNeuron(0),this->exo_rbf->getNeuron(2),1.0); // f(c) (input)
	w(this->dmp->getInputNeuron(1),this->inputNeurons->getNeuron(3),1.0); // goal
	w(this->dmp->getInputNeuron(2),this->inputNeurons->getNeuron(2),1.0); // theta

	
}

void EXO_JOINT_MODULE::init_gradn()
{
	this->gradn = new GRADN(this->params_ptr->kirnel);
	addSubnet(this->gradn);
	w(this->gradn->getInputNeuron(0),this->cpg[0]->getNeuron(0),1.0);
	w(this->gradn->getInputNeuron(1),this->cpg[0]->getNeuron(1),1.0);
	w(this->gradn->getInputNeuron(2),this->dmp->getErrorNeuron(),1.0);

	for(int i=0;i<this->params_ptr->kirnel;i++){
		w(this->gradn->getInputNeuron(3+i),this->exo_rbf->getNeuron(4+i),1.0);
		this->gradn->setCenter(i,this->exo_rbf->getCenter(i,0),this->exo_rbf->getCenter(i,1));
		this->gradn->setWeight(i,this->exo_rbf->getWeight(i));
	}
}

void EXO_JOINT_MODULE::addNoise()
{
	default_random_engine generator;
	normal_distribution<double> distribution(0.0,this->params_ptr->noise);
  	for(int i =0;i<this->params_ptr->kirnel;i++)
  	{
  		this->man_rbf->setWeight(i,this->man_rbf->getWeight(i)+distribution(generator));
  	}
}

void EXO_JOINT_MODULE::getTarget(){

	ifstream myFile(this->params_ptr->filename);
	cout << this->params_ptr->filename << endl;
    if (1-myFile.good()){return;}

	string myText; // buffer (string)
	std::vector <float> data_; // buffer (string -> float)

	float min = 1000; // lowest point
	float max = -1000; // highest point

	float current = 0.0; // data
	float smooth_current = 0.0; // filtered data
	float previous = 0.0; // previous filtered data
	int count = 0; // counter


	while (getline (myFile, myText)) {
		current = stof(myText);
		data_.push_back(current);
	}
	myFile.close();

	//ROS_INFO_STREAM("length of the reference trajectory: " << data_.size());

	// position trajectory
	for (int i=0;i<TARGETLENGTH;i++){
		this->position_trajectory[i] = (3.14/180.0)*data_[int(i*data_.size()/TARGETLENGTH)];
		if (this->position_trajectory[i] < min){min = this->position_trajectory[i];}
		if (this->position_trajectory[i] > max){max = this->position_trajectory[i];}

	}
	// velocity trajectory
	float velocity_trajectory[TARGETLENGTH] = {0.0};
	for (int i=1;i<TARGETLENGTH;i++){velocity_trajectory[i] = (this->position_trajectory[i]-this->position_trajectory[i-1])/this->params_ptr->dt_target;} // v = dx/t
	velocity_trajectory[0] = (this->position_trajectory[0]-this->position_trajectory[TARGETLENGTH-1])/this->params_ptr->dt_target;
	// acceleration trajectory
	float acceleration_trajectory[TARGETLENGTH] = {0.0};
	for (int i=1;i<TARGETLENGTH;i++){acceleration_trajectory[i] = (velocity_trajectory[i]-velocity_trajectory[i-1])/this->params_ptr->dt_target;} // a = dv/t
	acceleration_trajectory[0] = (acceleration_trajectory[0]-acceleration_trajectory[TARGETLENGTH-1])/this->params_ptr->dt_target;
	
	// inverse DMP equations to find the reference
	this->goal = 0.5*(max+min);
	float z =0;
	float msd = 0;
	for (int i=0;i<TARGETLENGTH;i++){
		z = this->params_ptr->dt_target*acceleration_trajectory[i]; // z = a*dt
		msd = this->params_ptr->alpha*(this->params_ptr->beta*(this->goal-this->position_trajectory[i])-z); // z = a(b(g-y)-z)
		this->target_trajectory[i] = velocity_trajectory[i]-this->params_ptr->dt_target*msd; // f(c) = v-a(b(g-y)-z)*dt
	}

}//EXO_JOINT_MODULE::getTarget()