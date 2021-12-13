#ifndef EXO_JOINT_MODULE_PARAMETERS_H
#define EXO_JOINT_MODULE_PARAMETERS_H

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

struct exoJointModuleParams
{
	string name;
	string filename;

	int mode = 0; // mode 0 = positional mode , x = f(c) + dx*dt // mode 1 = velocity mode , x = x + f(c)*dt + v*dt
	int phase_shift = -10;  // phase shift
	
	float alpha = 4.0;
	float beta = 1.0;
	float cd = 1.8;
	float cp = 2.5;
	float eta_rbf = 1.0;
	float eta_cpg = 0;
	float eta_dphi = 0;
	float gamma_primary = 1.01;
	float gamma_secondary = 1.01;
	float kirnel = 32;
	float kd = 0.8;
	float kp = 1.2;
	float noise = 0.00;
	float phi_primary = 0.04926;
	float phi_secondary = 0.04926;
	float sigma_primary = 0.04;
	float sigma_secondary = 0.04;
	float dt_target = 0.01;
	float dt = 0.95*dt_target*phi_primary/0.04926;
};


#endif //EXO_JOINT_MODULE_PARAMETERS_H