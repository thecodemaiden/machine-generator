//
//  NEATSpatialRotation.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/15/13.
//
//

#ifndef __SystemGenerator__NEATSpatialRotation__
#define __SystemGenerator__NEATSpatialRotation__

#include "NEATSpatialAlgorithm.h"

class NEATSpatialRotation : public NEATSpatialAlgorithm {
    
    MachineSystem *createInitialSystem();
    void prepareInitialPopulation();
    void stepSystem(SystemInfo *individual);
    
    cpFloat evaluateSystem(SystemInfo *sys);
    bool goodEnoughFitness(cpFloat bestFitness);
public:
    NEATSpatialRotation(int populationSize, int maxGenerations, int maxStagnation, float p_c=0.3, float p_m_attach=0.2, float p_m_node=0.1, float p_m_conn=0.1);
    
    virtual  char* inputDescription();
    virtual  char* outputDescription();
};

#endif /* defined(__SystemGenerator__NEATSpatialRotation__) */
