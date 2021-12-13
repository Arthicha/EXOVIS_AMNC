#include "rbf.h"

RBF::RBF(int n_kirnel)
{
	this->n_kirnel = n_kirnel;

	this->weights.clear();
	for(int i=0;i<this->n_kirnel;i++){this->weights.push_back(0.0);}
	for(int j=0;j<2;j++)
	{
		this->centers[j].clear();
		for(int i=0;i<this->n_kirnel;i++){this->centers[j].push_back(0.0);}
	}

	setNeuronNumber(4+this->n_kirnel);
	for(int i=0;i<(4+this->n_kirnel);i++){setTransferFunction(i,identityFunction());}
}


void RBF::setLearningRate(double x){
	this->eta = x;
}

void RBF::setTrainingIteration(int x){
	this->training_iteration = x;
}

void RBF::setTrainingPeriod(int x){
	this->training_period = x;
	this->target.clear();
	for(int i=0;i<this->training_period;i++){this->target.push_back(0.0);}
}

void RBF::setTrainingSkip(int x){
	this->training_skip = x;
}

void RBF::setSigma(double x){
	this->sigma = x;
}

void RBF::setPhi(double x){
	this->phi = x;
}

void RBF::setGamma(double x){
	this->gamma = x;
}

void RBF::setPresteps(int x){
	this->presteps = x;
}

double RBF::fit(float array[])
{
	for(int i=0;i<this->training_period;i++){this->target[i] = array[i];}

	// initialize cpg
	so2cpg = new SO2CPG();

	//destabilize cpg to oscillate
	so2cpg->setOutput(1, 0.885);so2cpg->setOutput(0, -0.59268);
	so2cpg->setActivity(0, 0.885);so2cpg->setActivity(1, -0.59268);

	//set cpg weights to override timos phi weight matrix
	w(so2cpg->getNeuron(0), so2cpg->getNeuron(0), this->gamma*cos(this->phi));w(so2cpg->getNeuron(0), so2cpg->getNeuron(1), this->gamma*sin(this->phi));
	w(so2cpg->getNeuron(1), so2cpg->getNeuron(0), -this->gamma*sin(this->phi));w(so2cpg->getNeuron(1), so2cpg->getNeuron(1), this->gamma*cos(this->phi));
	so2cpg->setTransferFunction(so2cpg->getNeuron(0),tanhFunction());
	so2cpg->setTransferFunction(so2cpg->getNeuron(1),tanhFunction());

	//set bias
	so2cpg->setBias(0, 0.0);so2cpg->setBias(1, 0.0);
	addSubnet(so2cpg);


	for(int i=0;i<this->presteps;i++){
		so2cpg->step();
	}
	for(int i =0;i<this->training_period;i++){for(int j=0;j<2;j++){training_cpg[j].push_back(so2cpg->getOutput(j));} so2cpg->step();}

	// calculate the centers : cx, cy (radious of each RBF)
	double step = this->training_period/this->n_kirnel;

	for(int i=0;i<this->n_kirnel;i++){for(int j=0;j<2;j++){centers[j][i]=training_cpg[j][int(i*step)];}}

	// traing RBF network for n iteration

	double error = 0.0;
	this->previous_delta_weights.clear();
	for(int i=0;i<this->n_kirnel;i++){this->previous_delta_weights.push_back(0.0);}
	
	for(int n=0;n<this->training_iteration;n++)
	{
		error = 0.0;
		this->delta_weights.clear();
		for(int i=0;i<this->n_kirnel;i++){this->delta_weights.push_back(0.0);}
		// for each time step
		for(int t=0;t<this->training_period;t+=this->training_skip)
		{
			float output = 0.0;
			for(int i=0;i<this->n_kirnel;i++){output += this->weights[i]*this->basisFunction(this->training_cpg[0][t],this->training_cpg[1][t],i);}
			for(int i=0;i<this->n_kirnel;i++){
				this->delta_weights[i] += (this->target[t]-output)*this->basisFunction(this->training_cpg[0][t],this->training_cpg[1][t],i);
			}
			error += abs(this->target[t]-output);
		}
		for(int i=0;i<this->n_kirnel;i++){this->weights[i] += this->eta*(this->delta_weights[i]+0.9*this->previous_delta_weights[i])/this->training_period;}
		for(int i=0;i<this->n_kirnel;i++){this->previous_delta_weights[i] = this->delta_weights[i];}
	}
	
	return error;
	
}

void RBF::updateActivities()
{
	this->getNeuron(0)->updateActivity();
	this->getNeuron(1)->updateActivity();
	float output = 0.0;
	float ki = 0.0;

	for(int k=0;k<this->n_kirnel;k++) {
		ki = this->basisFunction(this->getOutput(0),this->getOutput(1),k);
		output += this->weights[k]*ki;
		this->setActivity(4+k,ki);
	}
	this->setActivity(2,output);
	this->setActivity(3,this->target[count]);
}

void RBF::step(){

	updateActivities();
	updateOutputs();
	for(int i=0;i<2;i++){setOutput(i,getOutput(i));setActivity(i,getOutput(i));}

	float output = 0.0;
	float ki = 0.0;
	for(int k=0;k<this->n_kirnel;k++) {
		ki = basisFunction(getOutput(0),getOutput(1),k);
		output += this->weights[k]*ki;
		setOutput(4+k,ki);setActivity(4+k,ki);
	}
			
	setOutput(2,output);setActivity(2,output);
	setOutput(3,this->target[count]);setActivity(3,this->target[count]);
	count += 1;if(count >= this->training_period){count = 0;}
}

double RBF::getGradient(int cpg_num, float phi)
{
	float grad = 0.0;
	for(int i=0;i<this->n_kirnel;i++)
	{
		if (cpg_num == 0)
		{
			grad += this->weights[i]*(getOutput(0)-centers[0][i]-(1-pow(getOutput(1),2))*(getOutput(1)-centers[1][i])*sin(phi))*basisFunction(getOutput(0),getOutput(1),i);
		}else
		{
			grad += this->weights[i]*(getOutput(1)-centers[1][i]+(1-pow(getOutput(0),2))*(getOutput(0)-centers[0][i])*cos(phi))*basisFunction(getOutput(0),getOutput(1),i);
		}
	}
	return grad;
}

double RBF::getCenter(int kirnel_num, int cpg_num)
{
	return this->centers[cpg_num][kirnel_num];
}

double RBF::getWeight(int weight_num)
{
	return this->weights[weight_num];
}

void RBF::setWeight(int weight_num, float value)
{
	this->weights[weight_num] = value;
}


double RBF::basisFunction(double inputx, double inputy ,int num)
{
	return exp(-1.0*(pow(inputx - this->centers[0][num],2)+pow(inputy - this->centers[1][num],2))/this->sigma);
}

