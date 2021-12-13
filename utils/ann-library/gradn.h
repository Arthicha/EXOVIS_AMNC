#ifndef GRADN_H_
#define GRADN_H_

#include "ann.h"
#include "neuron.h"
#include "pmn.h"

#define FULLCOMPUTATION 0


class Neuron;
class PMN;

class GRADN : public ANN {
public:
    
    GRADN(int n_kirnel);

    Neuron* getInputNeuron(int index);
    Neuron* getOutputNeuron(int index);

    float getGradient(int index);

    void setCenter(int kirnel_num, float value_x, float value_y);
    void setWeight(int weight_num, float value);
    void setParams(float phi, float gamma);

    void step();
    void updateActivities();


private:

	int n_kirnel = 32;

	float phi = 0.04926;
	float gamma = 1.01;
	float gradient[2] = {0.0};
	float phi_gradient = 0.0;

	PMN* inputNeurons;
	PMN* outputNeurons;

	vector<float> centers[2];
	vector<float> weights;

	vector<float> psi;

	void computeGradient();
	void init_internal_parameters();

	double gaussianFunction(double x, double c ,float sigma);




};


#endif /* GRADN_H_ */
