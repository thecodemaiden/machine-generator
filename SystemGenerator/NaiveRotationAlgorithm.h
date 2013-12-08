//
//  NaiveBadAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/7/13.
//
//

#ifndef __SystemGenerator__NaiveBadAlgorithm__
#define __SystemGenerator__NaiveBadAlgorithm__

#include "NaiveBaseAlgorithm.h"


class NaiveRotationAlgorithm : public NaiveAlgorithm {
    
     MachineSystem *createInitialSystem();
      MachineSystem *mutateSystem(MachineSystem *original);
     void stepSystem(SystemInfo *individual);

     cpFloat evaluateSystem(SystemInfo *sys);
     bool goodEnoughFitness(cpFloat bestFitness);
    
    void performRecombinations();
    
    SystemInfo *bestIndividual;
    
public:
    cpFloat allTimeBestFitness;
    NaiveRotationAlgorithm(int populationSize=5, int maxGenerations = 100, int maxStagnation=5, float p_m=0.2, float p_c= 0.2);
    ~NaiveRotationAlgorithm();
    
    bool tick();
    
    MachineSystem *bestSystem();
    
     char* inputDescription();
     char* outputDescription();
};

#endif /* defined(__SystemGenerator__NaiveBadAlgorithm__) */

