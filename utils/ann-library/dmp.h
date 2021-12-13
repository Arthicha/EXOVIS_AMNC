#ifndef DMP_H_
#define DMP_H_

#include "ann.h"
#include "neuron.h"
#include "pmn.h"

#define TAU 1

class Neuron;
class PMN;

class DMP : public ANN {
public:
    
    DMP(int mode);
    void set_system_params(float a, float b);
    void set_time(float t);
    void set_impedance_params(float kp, float kd, float cp, float cd);
    void step();
    void updateMode(int mode);

    float getDMPOutput();
    Neuron* getOutputNeuron();
    Neuron* getInputNeuron(int i);
    Neuron* getImpedanceNeuron();
    Neuron* getErrorNeuron();


private:

	int mode = 0;
	// mode 0 = positional mode , x = f(c) + dx*dt
	// mode 1 = velocity mode , x = x + f(c)*dt + v*dt

	float alpha = 1.0;
	float beta = 1.0;
	float time = 0.0;
	float kp = 1.0;
	float kd = 1;
	float ck = 1;
	float cd = 1;
	float dt = 0.01;

	PMN* inputLayer;
	PMN* preprocessingLayer;
	PMN* impedanceLayer;
	PMN* stateLayer[3];

	void set_connections();

};


#endif /* DMP_H_ */
