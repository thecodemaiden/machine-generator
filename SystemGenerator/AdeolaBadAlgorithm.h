//
//  AdeolaBadAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/7/13.
//
//

#ifndef __SystemGenerator__AdeolaBadAlgorithm__
#define __SystemGenerator__AdeolaBadAlgorithm__

#include "Type1Algorithm.h"

struct SystemInfo
{
    // a machine system, its simulation space, and fitness info
    MachineSystem *system;
    cpSpace *space;
    double fitness;
    std::vector<float> outputValues;
    
    // we need to specify how many values this will hold for examination
    SystemInfo(int steps):outputValues(steps), fitness(0.0){}
};


class AdeolaAlgorithm : Type1Algorithm {
    std::vector<SystemInfo *> population;
    std::vector<float> lastInputValues;
    float p_m;
    float p_c = 0;
    int stepSize = 10;
    
    MachineSystem *createInitialSystem(cpSpace *space=NULL);
    MachineSystem *mutateSystem(MachineSystem *original);
    void combineSystems(MachineSystem *parent1, MachineSystem *parent2, MachineSystem **child1, MachineSystem **child2);
    cpFloat evaluateSystem(MachineSystem *sys);
    
    // 
    void afterWorldStep(cpSpace *space, void *key, void *data);
public:
    AdeolaAlgorithm(int populationSize=5);
    ~AdeolaAlgorithm();
    
    void tick();

};

#endif /* defined(__SystemGenerator__AdeolaBadAlgorithm__) */

