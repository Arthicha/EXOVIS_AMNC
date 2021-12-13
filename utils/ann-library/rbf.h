#ifndef RBF_H_
#define RBF_H_

#include <cmath>
#include "utils/ann-framework/ann.h"
#include "utils/ann-library/so2cpg.h"
#include "utils/ann-framework/transferfunction.h"
#include "utils/ann-framework/neuron.h"

using namespace std;

class RBF : public ANN {
public:

	
    RBF(int n_kirnel); //initialize RBF network

    // set RBF parameters
    void setLearningRate(double x);
    void setTrainingIteration(int x);
    void setTrainingPeriod(int x);
    void setTrainingSkip(int x);
    void setSigma(double x);

    // set internal cpg parameters
    void setPhi(double x);
    void setGamma(double x);
    void setPresteps(int x);

    // other functions
    double fit(float array[]);
    double getGradient(int cpg_num, float phi);
    double getCenter(int kirnel_num, int cpg_num);
    double getWeight(int weight_num);
    void setWeight(int weight_num, float value);
    void step(); // update rbf network
    void updateActivities();



private:

	SO2CPG * so2cpg; // internal cpg for training

	int count = 0;

	// radial basis function neural network parameter
	int n_kirnel = 32;
	int training_period = 128;
	int training_iteration = 500;
	int training_skip = 1; // iterate every n time step in the training process
	int training_shift = 40;
	double sigma = 0.01; // f(x) = e^(-x^2/s)
	double eta = 1.0; // learning rate

	// internal so2cpg parameter
	int presteps = 10000;
	double phi = 0.1;//0.04926;
	double gamma = 1.2;//1.01;

	// arrays
	vector<double> target;
	vector<double> training_cpg[2];
	vector<double> weights; 
	vector<double> delta_weights; 
	vector<double> previous_delta_weights; 
	vector<double> centers[2];


	void setup();
	double rbfFunction(double inputx,double inputy,double rx,double ry);
	double basisFunction(double inputx, double inputy, int num);

};


#endif /* PCPG_H_ */
