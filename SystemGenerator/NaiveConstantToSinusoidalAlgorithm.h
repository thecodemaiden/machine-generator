//
//  NaiveConstantToSinusoidalAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/31/13.
//
//

#ifndef __SystemGenerator__NaiveConstantToSinusoidalAlgorithm__
#define __SystemGenerator__NaiveConstantToSinusoidalAlgorithm__

#include "NaiveBaseAlgorithm.h"

class NaiveConstantToSinusoidalAlgorithm : public NaiveAlgorithm {
    
    MachineSystem *createInitialSystem();
    void stepSystem(SystemInfo *individual);
    
    cpFloat evaluateSystem(SystemInfo *sys);
    
    
public:
    cpFloat allTimeBestFitness;

    NaiveConstantToSinusoidalAlgorithm(int populationSize=5, int maxGenerations = 100, int maxStagnation=5, float p_m=0.2, float p_c= 0);
    ~NaiveConstantToSinusoidalAlgorithm();

    
    virtual char* inputDescription();
    virtual char* outputDescription();
};

#endif