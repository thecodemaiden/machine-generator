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
    
    cpFloat p_m_attach; // how likely are we to mutate a random attachment
    
    void stepSystem(SystemInfo *individual);
    
    cpFloat evaluateSystem(SystemInfo *sys);
    bool goodEnoughFitness(cpFloat bestFitness);
    
    void mutateSystem(MachineSystem *original); // also mutates the attachments themselves
    
public:
    NEATDisplacementToX(int populationSize, int maxGenerations, int maxStagnation, float p_c=0.5, float p_m_attach=0.25, float p_m_node=0.5, float p_m_conn=0.5);
    
    virtual  char* inputDescription();
    virtual  char* outputDescription();
};

#endif /* defined(__SystemGenerator__NEATDisplacementToX__) */
