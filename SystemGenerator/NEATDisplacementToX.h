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

class NEATDisplacementToX : public NEATAlgorithm {
    
    void stepSystem(SystemInfo *individual);
    
    cpFloat evaluateSystem(SystemInfo *sys);
    bool goodEnoughFitness(cpFloat bestFitness);
        
public:
    NEATDisplacementToX(int populationSize, int maxGenerations, int maxStagnation, float p_c=0.3, float p_m_attach=0.2, float p_m_node=0.1, float p_m_conn=0.1);
    
    virtual  char* inputDescription();
    virtual  char* outputDescription();
};

#endif /* defined(__SystemGenerator__NEATDisplacementToX__) */
