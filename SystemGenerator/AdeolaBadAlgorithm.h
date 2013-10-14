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
    // a machine system and fitness info
    MachineSystem *system;
    double fitness;
    std::vector<float> outputValues;
    std::vector<float> inputValues;

    // we need to specify how many values this will hold for examination
    SystemInfo(int steps): outputValues(steps), fitness(0.0),
                           inputValues(steps){}
    
    ~SystemInfo() {
        delete system;
    }
};


class AdeolaAlgorithm {
    std::vector<SystemInfo *> population;
    float p_m;
    float p_c = 0;
    int simSteps = 10;
    
     MachineSystem *createInitialSystem();
     MachineSystem *mutateSystem(MachineSystem *original);

    cpFloat evaluateSystem(SystemInfo *sys);
    
    SystemInfo *bestIndividual;
    
    void stepSystem(SystemInfo *individual);

public:
    spaceUpdateFunc updateFunction;
    
    AdeolaAlgorithm(int populationSize=5, float p_m=0.02);
    ~AdeolaAlgorithm();
    
    void tick();
    
    MachineSystem *bestSystem();
};

#endif /* defined(__SystemGenerator__AdeolaBadAlgorithm__) */

