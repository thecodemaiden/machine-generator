//
//  AdeolaConstantToSinusoidalAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/31/13.
//
//

#ifndef __SystemGenerator__AdeolaConstantToSinusoidalAlgorithm__
#define __SystemGenerator__AdeolaConstantToSinusoidalAlgorithm__

#include "AdeolaBaseAlgorithm.h"

class AdeolaConstantToSinusoidalAlgorithm : public AdeolaAlgorithm {
    
    MachineSystem *createInitialSystem();
    virtual MachineSystem *mutateSystem(MachineSystem *original);
    virtual void stepSystem(SystemInfo *individual);
    
    virtual cpFloat evaluateSystem(SystemInfo *sys);
    virtual bool goodEnoughFitness(cpFloat bestFitness);
    
    std::vector<SystemInfo *> population;
    
    SystemInfo *bestIndividual;
    
public:
    cpFloat allTimeBestFitness;

    AdeolaConstantToSinusoidalAlgorithm(int populationSize=5, int maxGenerations = 100, int maxStagnation=5, float p_m=0.2, float p_c= 0);
    ~AdeolaConstantToSinusoidalAlgorithm();
    
    bool tick();
    
    MachineSystem *bestSystem();
    
    virtual char* inputDescription();
    virtual char* outputDescription();
};

#endif