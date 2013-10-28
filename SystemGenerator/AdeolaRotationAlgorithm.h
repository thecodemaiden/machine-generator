//
//  AdeolaBadAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/7/13.
//
//

#ifndef __SystemGenerator__AdeolaBadAlgorithm__
#define __SystemGenerator__AdeolaBadAlgorithm__

#include "AdeolaBaseAlgorithm.h"


class AdeolaRotationAlgorithm : public AdeolaAlgorithm {
    
     MachineSystem *createInitialSystem();
     MachineSystem *mutateSystem(MachineSystem *original);
    void stepSystem(SystemInfo *individual);

    cpFloat evaluateSystem(SystemInfo *sys);    
    bool goodEnoughFitness(cpFloat bestFitness);
    
    std::vector<SystemInfo *> population;
    
    SystemInfo *bestIndividual;
    
public:
    cpFloat allTimeBestFitness;
    AdeolaRotationAlgorithm(int populationSize=5, int maxGenerations = 100, int maxStagnation=5, float p_m=0.2, float p_c= 0);
    ~AdeolaRotationAlgorithm();
    
    bool tick();
    
    MachineSystem *bestSystem();
    
    char* inputDescription();
    char* outputDescription();
};

#endif /* defined(__SystemGenerator__AdeolaBadAlgorithm__) */

