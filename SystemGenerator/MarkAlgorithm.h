//
//  MarkAlgorithm.h
//  SystemGenerator
//
//  Created by Mark Whiting on 10/18/13, based onf a file by Adeola Bannis.
//
//

#ifndef __SystemGenerator__MarkAlgorithm__
#define __SystemGenerator__MarkAlgorithm__

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


class MarkAlgorithm {
    std::vector<SystemInfo *> population;
    float p_m;
    float p_c = 0;
    int simSteps = 10;
    int stagnantGenerations = 0;
    int maxStagnation;
    
    MachineSystem *createInitialSystem();
    MachineSystem *mutateSystem(MachineSystem *original);
    
    cpFloat evaluateSystem(SystemInfo *sys);
    
    SystemInfo *bestIndividual;
    
    void stepSystem(SystemInfo *individual);
    
public:
    spaceUpdateFunc updateFunction;
    
    MarkAlgorithm(int populationSize=5, float p_m=0.02, int maxStagnation=5);
    ~MarkAlgorithm();
    
    bool tick(); // returns true if best fitness has been stagnant
    
    MachineSystem *bestSystem();
};

#endif /* defined(__SystemGenerator__MarkAlgorithm__) */

