//
//  AdeolaBaseAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/14/13.
//
//

#ifndef __SystemGenerator__AdeolaBaseAlgorithm__
#define __SystemGenerator__AdeolaBaseAlgorithm__

#include "Algorithm.h"
#include <string>


struct SystemInfo
{
    // a machine system and fitness info
    MachineSystem *system;
    double fitness;
    std::vector<cpFloat> outputValues;
    std::vector<cpFloat> inputValues;
    
    // we need to specify how many values this will hold for examination
    SystemInfo(int steps): outputValues(steps), fitness(0.0),
    inputValues(steps){}
    
    ~SystemInfo() {
        delete system;
    }
};

class AdeolaAlgorithm {
protected:
    
    float p_m;
    float p_c = 0;
    int simSteps = 10;
    int stagnantGenerations = 0;
    int maxStagnation;
    int maxGenerations;
    
    long generations;
    
public:
    AdeolaAlgorithm(int maxGenerations, int maxStagnation, float p_m, float p_c);
    virtual ~AdeolaAlgorithm() {};
    
    virtual bool tick() = 0; // returns true if we should stop iterating - please override
    
    virtual MachineSystem *bestSystem() = 0; // please override according to the representation for your algorithm
    virtual long getNumberOfIterations();
    
    // more functions to override
    virtual  char* inputDescription();
    virtual  char* outputDescription();
};

#endif /* defined(__SystemGenerator__AdeolaBaseAlgorithm__) */
