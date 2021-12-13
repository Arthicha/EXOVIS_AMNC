#include "gradn.h"


GRADN::GRADN(int n_kirnel)
{
    this->n_kirnel = n_kirnel;

    this->init_internal_parameters();

    /* input neurons are: (0) C0 (1) C1 (2) error (3 - (3+n_kirnel)) radial basis k*/
    this->inputNeurons = new PMN(3+this->n_kirnel);
    addSubnet(this->inputNeurons);
    for(int i=0;i<(3+this->n_kirnel);i++){setTransferFunction(this->inputNeurons->getNeuron(i),identityFunction());}

    /* output neurons are: (0) partial(error)/partial(C0) and (1) partial(error)/partial(C1)*/
    this->outputNeurons = new PMN(3);
    addSubnet(this->outputNeurons);
    for(int i=0;i<3;i++){setTransferFunction(this->outputNeurons->getNeuron(i),identityFunction());}
}

void GRADN::computeGradient()
{

    this->gradient[0] = 0.0;this->gradient[1] = 0.0;
    this->phi_gradient = 0.0;
    float cpg[2] = {0.0};
    float error = 1.0*this->inputNeurons->getOutput(2);

    for(int i=0;i<2;i++){cpg[i] = this->inputNeurons->getOutput(i);}

    for(int k=0;k<this->n_kirnel;k++){this->psi[k]=this->inputNeurons->getOutput(3+k);}

    for(int k=0;k<this->n_kirnel;k++)
    {
        float lambda = 0.00;
        this->gradient[0] += this->weights[k]*error*(cpg[0]-this->centers[0][k])*(1-pow(cpg[0],2))*this->psi[k];

        this->gradient[1] += this->weights[k]*error*(cpg[1]-this->centers[1][k])*(1-pow(cpg[1],2))*this->psi[k];
    }
    setActivity(this->outputNeurons->getNeuron(0),this->gradient[0]);

    setActivity(this->outputNeurons->getNeuron(1),this->gradient[1]);

    setActivity(this->outputNeurons->getNeuron(2),this->phi_gradient);


}

void GRADN::step()
{
    this->inputNeurons->step();
    this->computeGradient();
    this->outputNeurons->updateOutputs();
}

void GRADN::updateActivities()
{
    this->inputNeurons->updateActivities();
    this->outputNeurons->updateActivities();
    this->computeGradient();
}

Neuron* GRADN::getInputNeuron(int index)
{
    return this->inputNeurons->getNeuron(index);
}

Neuron* GRADN::getOutputNeuron(int index)
{
    return this->outputNeurons->getNeuron(index);
}


float GRADN::getGradient(int index)
{
    return this->outputNeurons->getOutput(index);
}

void GRADN::setCenter(int kirnel_num, float value_x, float value_y)
{
    this->centers[0][kirnel_num] = value_x;
    this->centers[1][kirnel_num] = value_x;
}

void GRADN::setParams(float phi, float gamma)
{
    this->phi = phi;
    this->gamma = gamma;
}

void GRADN::setWeight(int weight_num, float value)
{
    this->weights[weight_num] = value;
}


double GRADN::gaussianFunction(double x, double c ,float sigma)
{
    return exp(-1.0*(pow(x - c,2))/sigma);
}

void GRADN::init_internal_parameters()
{
    this->weights.clear();
    this->psi.clear();
    for(int j=0;j<2;j++){this->centers[j].clear();}

    for(int i=0;i<this->n_kirnel;i++)
    {
        for(int j=0;j<2;j++){this->centers[j].push_back(0.0);}
        this->weights.push_back(0.0);
        this->psi.push_back(0.0);
    }
}

