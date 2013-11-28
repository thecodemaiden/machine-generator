//
//  NEATDisplacementToX.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/4/13.
//
//

#ifndef __SystemGenerator__NEATDisplacementToX__
#define __SystemGenerator__NEATDisplacementToX__

#include "NEATAlgorithm.h"

class NEATRotation : public NEATAlgorithm {
    void prepareInitialPopulation();
    void stepSystem(ExtendedSystemInfo *individual);
    
    cpFloat evaluateSystem(ExtendedSystemInfo *sys);
    bool goodEnoughFitness(cpFloat bestFitness);
        
public:
    NEATRotation(int populationSize, int maxGenerations, int maxStagnation, float p_c=0.3, float p_m_attach=0.2, float p_m_node=0.1, float p_m_conn=0.1);
    
    virtual  char* inputDescription();
    virtual  char* outputDescription();
};

#endif /* defined(__SystemGenerator__NEATDisplacementToX__) */
