#include "dmp.h"


DMP::DMP(int mode)
{
    this->mode = mode;
    this->inputLayer = new PMN(4); // (0) F(c) (1) goal (2) theta (3) delayed theta
    addSubnet(this->inputLayer);
    for(int i=0;i<4;i++){setTransferFunction(this->inputLayer->getNeuron(i),identityFunction());}

    this->preprocessingLayer = new PMN(2+TAU); // (0) command (1) delayed command
    addSubnet(this->preprocessingLayer);
    for(int i=0;i<2+TAU;i++){setTransferFunction(this->preprocessingLayer->getNeuron(i),identityFunction());}

    this->impedanceLayer = new PMN(3); // (0) impedance control I (1) delayed I (2) positional error
    addSubnet(this->impedanceLayer);
    for(int i=0;i<3;i++){setTransferFunction(this->impedanceLayer->getNeuron(i),identityFunction());}

    for(int i =0;i<3;i++)
    {
        this->stateLayer[i] = new PMN(1);
        addSubnet(this->stateLayer[i]);
        setTransferFunction(this->stateLayer[i]->getNeuron(0),identityFunction());
    }
    
    //this->init_static_connections();
    this->set_connections();
}

void DMP::step()
{
    this->inputLayer->step();
    this->preprocessingLayer->step();
    this->impedanceLayer->step();
    for(int i =0;i<3;i++){this->stateLayer[i]->step();}
}

void DMP::updateMode(int mode)
{
    this->mode = mode;
    if (this->mode == 0)
    {
        w(this->stateLayer[2]->getNeuron(0),this->inputLayer->getNeuron(0),1.0); // theta = f(c)
        w(this->stateLayer[2]->getNeuron(0),this->stateLayer[2]->getNeuron(0),0.0); 
    }
    if (this->mode == 1)
    {
        w(this->stateLayer[2]->getNeuron(0),this->stateLayer[2]->getNeuron(0),1.0); // theta = theta + ...
        w(this->stateLayer[2]->getNeuron(0),this->inputLayer->getNeuron(0),this->dt);
    }

}

void DMP::set_connections()
{
    // theta*[t-1] = delay(theta[t])
    w(this->inputLayer->getNeuron(3),this->inputLayer->getNeuron(2),1.0); // delayed theta feedback
    // I[t-1] = delay(I[t])
    w(this->impedanceLayer->getNeuron(1),this->impedanceLayer->getNeuron(0),1); // delayed impedanced control
    // delay theta[t] -> feedback delay/response delay
    w(this->preprocessingLayer->getNeuron(0),this->stateLayer[2]->getNeuron(0),1.0); // control command
    for(int i = 1;i<2+TAU;i++){w(this->preprocessingLayer->getNeuron(i),this->preprocessingLayer->getNeuron(i-1),1.0);}
    // I = kp(theta_fb - theta) + kd(dtheta_Fb-dtheta)
    w(this->impedanceLayer->getNeuron(0),this->inputLayer->getNeuron(2),this->kp); 
    w(this->impedanceLayer->getNeuron(0),this->preprocessingLayer->getNeuron(0+TAU),-this->kp); 
    w(this->impedanceLayer->getNeuron(0),this->inputLayer->getNeuron(3),-this->kd); 
    w(this->impedanceLayer->getNeuron(0),this->preprocessingLayer->getNeuron(1+TAU),this->kd); 
    // E = kp(theta_fb - theta)
    w(this->impedanceLayer->getNeuron(2),this->inputLayer->getNeuron(2),this->kp); 
    w(this->impedanceLayer->getNeuron(2),this->preprocessingLayer->getNeuron(0+TAU),-this->kp); 
    // acceleration = alpha(beta(goal-theta)-dtheta) + cd*(I[t]-I[t-1])
    w(this->stateLayer[0]->getNeuron(0),this->inputLayer->getNeuron(1),this->beta*this->alpha); 
    w(this->stateLayer[0]->getNeuron(0),this->stateLayer[2]->getNeuron(0),-this->beta*this->alpha); 
    w(this->stateLayer[0]->getNeuron(0),this->stateLayer[1]->getNeuron(0),-this->alpha); 
    w(this->stateLayer[0]->getNeuron(0),this->impedanceLayer->getNeuron(0),this->cd); //ck*I[t]
    w(this->stateLayer[0]->getNeuron(0),this->impedanceLayer->getNeuron(1),-this->cd); //ck*I[t-1]
    // velocity =  velocity + acceleration*dt + ck*I  + ...
    w(this->stateLayer[1]->getNeuron(0),this->stateLayer[1]->getNeuron(0),1.0); // v = v + ...
    w(this->stateLayer[1]->getNeuron(0),this->stateLayer[0]->getNeuron(0),this->dt); // ddtheta*dt
    w(this->stateLayer[1]->getNeuron(0),this->impedanceLayer->getNeuron(0),this->ck); //ck*I
    // position = velocity*dt + ...
    w(this->stateLayer[2]->getNeuron(0),this->stateLayer[1]->getNeuron(0),this->dt); // dtheta*dt
    if (this->mode == 1)
    {
        (this->stateLayer[2]->getNeuron(0),this->stateLayer[2]->getNeuron(0),1.0); // theta = theta + ...
        w(this->stateLayer[2]->getNeuron(0),this->inputLayer->getNeuron(0),this->dt); // theta = f(c)*dt + ...
    }else if (this->mode == 0){
        w(this->stateLayer[2]->getNeuron(0),this->inputLayer->getNeuron(0),1.0); // theta = f(c)
    }
}


void DMP::set_system_params(float a, float b)
{
    this->alpha = a;
    this->beta = b;
    this->set_connections();
}

void DMP::set_time(float t)
{
    this->dt = t;
    this->set_connections();
}

void DMP::set_impedance_params(float kp, float kd, float cp, float cd)
{
    this->kp = kp;
    this->kd = kd;
    this->ck = cp;
    this->cd = cd;
    this->set_connections();
}

float DMP::getDMPOutput()
{
    return this->stateLayer[2]->getOutput(0);
}

Neuron* DMP::getOutputNeuron()
{
    return this->stateLayer[2]->getNeuron(0);
}

Neuron* DMP::getInputNeuron(int i)
{
    return this->inputLayer->getNeuron(i);
}

Neuron* DMP::getImpedanceNeuron()
{
    return this->impedanceLayer->getNeuron(0);
}

Neuron* DMP::getErrorNeuron()
{
    return this->impedanceLayer->getNeuron(2);
}

