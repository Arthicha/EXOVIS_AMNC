#ifndef EXO_JOINT_MODULE_H
#define EXO_JOINT_MODULE_H

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <map>
#include <ros/ros.h>
#include <random>
#include "dmp.h"
#include "rbf.h"
#include "pmn.h"
#include "gradn.h"
#include "neuron.h"
#include "exo_joint_module_parameters.h"

#define TARGETLENGTH 128
#define TRAINING_ITERATION 10000
#define TRAINING_SKIP 4
#define TRAINING_PHI 0.04926
#define TRAINING_GAMMA 1.01
#define TRAINING_PRESTEP 10000

using namespace std;

class EXO_JOINT_MODULE: public ANN{
public:
	EXO_JOINT_MODULE(struct exoJointModuleParams* ptr);

    void setInput(int i, float value);
    
    Neuron* getInputNeuron(int i);
	Neuron* getExoOutputNeuron();
	Neuron* getManOutputNeuron();
	Neuron* getCPGNeuron(int cpg_num, int neuron_num);
	Neuron* getGradNeuron(int index);
	float getExoOutput();
	float getManOutput();
	float getCPG(int cpg_num, int neuron_num);
	float getDebug(int id);

    void step();
    void updateActivities();
    void updateOutputs();
    void updateNewParams();
    void updateMode();

	

private:

	struct exoJointModuleParams* params_ptr;
	
	PMN* cpg[2];
	PMN* inputNeurons;
	RBF* exo_rbf;
	RBF* man_rbf;
	DMP* dmp;
	GRADN* gradn;

	float goal = 0.0;

	float position_trajectory[TARGETLENGTH] = {0.0};
	float target_trajectory[TARGETLENGTH] = {0.0};

	
	

	
	void init_cpg(int module);
	void init_ipn();
	void init_exo_rbf();
	void init_man_rbf();
	void init_dmp();
	void init_gradn();


	void set_cpg_connection(int module);
	void reset_cpg_activity(int module);
	void addNoise();

	void getTarget();

	void cpg_step(int cpg_num, int timesteps);

};







#endif // EXO_JOINT_MODULE_H