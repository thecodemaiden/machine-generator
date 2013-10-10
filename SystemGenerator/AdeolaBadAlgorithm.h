//
//  AdeolaBadAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/7/13.
//
//

#ifndef __SystemGenerator__AdeolaBadAlgorithm__
#define __SystemGenerator__AdeolaBadAlgorithm__

#include "Algorithm.h"

struct SystemInfo
{
    // a machine system, its simulation space, and fitness info
    MachineSystem *system;
    cpSpace *space;
    double fitness;
    std::vector<float> outputValues;
    std::vector<float> inputValues;

    // we need to specify how many values this will hold for examination
    SystemInfo(int steps): outputValues(steps), fitness(0.0),
                           inputValues(steps){}
};


class AdeolaAlgorithm {
    std::vector<SystemInfo *> population;
    float p_m;
    float p_c = 0;
    int simSteps = 10;
    
     MachineSystem *createInitialSystem(cpSpace *space=NULL);
     MachineSystem *mutateSystem(MachineSystem *original);

    cpFloat evaluateSystem(SystemInfo *sys);
    
    SystemInfo *bestIndividual;

public:
    spaceUpdateFunc updateFunction;
    
    AdeolaAlgorithm(int populationSize=5, float p_m=0.1);
    ~AdeolaAlgorithm();
    
    void tick();
    
    cpSpace *simulationSpaceForBestIndividual();
    MachineSystem *bestSystem();
};

#endif /* defined(__SystemGenerator__AdeolaBadAlgorithm__) */

