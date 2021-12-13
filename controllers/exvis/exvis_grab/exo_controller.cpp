// 
// created by zumo arthicha.
// 

#include "neural_controller.h"

int main(int argc,char* argv[])
{
    neuralController controller(argc,argv);
    while(true)
    {
    	if(not controller.runController()) break;
    }
    return(0);
}