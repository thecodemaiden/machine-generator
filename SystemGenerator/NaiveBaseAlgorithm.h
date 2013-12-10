//
//  NaiveBaseAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/14/13.
//
//

#ifndef __SystemGenerator__NaiveBaseAlgorithm__
#define __SystemGenerator__NaiveBaseAlgorithm__

#include "Algorithm.h"
#include <string>
#include <fstream>

class NaiveAlgorithm {
protected:
    
    float p_m;
    float p_c = 0;
    int simSteps = 10;
    int stagnantGenerations = 0;
    int maxStagnation;
    int maxGenerations;
    
    long generations;
    
    cpFloat allTimeBestFitness;
    
    std::ofstream currentLogFile;
    std::vector<SystemInfo *> population;
    SystemInfo *bestIndividual;

    std::vector<SystemInfo *> spawnChildren();
    
    virtual MachineSystem *createInitialSystem() = 0;
    virtual cpFloat evaluateSystem(SystemInfo *sys) = 0;

    virtual MachineSystem *mutateSystem(MachineSystem *original);
    virtual void stepSystem(SystemInfo *individual) = 0;
    
    bool goodEnoughFitness(cpFloat bestFitness);
    
    
public:
    NaiveAlgorithm(int maxGenerations, int maxStagnation, float p_m, float p_c);
    virtual ~NaiveAlgorithm();
    
    virtual bool tick(); // returns true if we should stop iterating - please override
    
    virtual MachineSystem *bestSystem(); // please override according to the representation for your algorithm
    virtual long getNumberOfIterations();
    
    // more functions to override
    virtual  char* inputDescription();
    virtual  char* outputDescription();
    
    void logPopulationStatistics();
};

#endif /* defined(__SystemGenerator__NaiveBaseAlgorithm__) */
