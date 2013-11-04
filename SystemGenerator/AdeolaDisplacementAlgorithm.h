//
//  AdeolaDisplacementAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/14/13.
//
//

#ifndef __SystemGenerator__AdeolaDisplacementAlgorithm__
#define __SystemGenerator__AdeolaDisplacementAlgorithm__

#include "AdeolaBaseAlgorithm.h"

// ok harder - I am going to rotate the input and I want the output to move, not rotate
// going to try recombination as well

class AdeolaDisplacementAlgorithm : public AdeolaAlgorithm {
    MachineSystem *createInitialSystem();
    MachineSystem *mutateSystem(MachineSystem *original);
    void stepSystem(SystemInfo *individual);
    
    cpFloat evaluateSystem(SystemInfo *sys);
    bool goodEnoughFitness(cpFloat bestFitness);
    
    std::vector<SystemInfo *> population;
        
    SystemInfo *bestIndividual;
    
public:
    
    AdeolaDisplacementAlgorithm(int populationSize=5, int maxGenerations = 100, int maxStagnation=5, float p_m=0.02, float p_c= 0);
    ~AdeolaDisplacementAlgorithm();
    
    bool tick();
    
    MachineSystem *bestSystem();
    
    char* inputDescription();
    char* outputDescription();

};

#endif /* defined(__SystemGenerator__AdeolaDisplacementAlgorithm__) */
