//
//  NEATSpatialSinRotation.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/25/13.
//
//

#ifndef __SystemGenerator__NEATSpatialSinRotation__
#define __SystemGenerator__NEATSpatialSinRotation__

#include "NEATSpatialAlgorithm.h"

class NEATSpatialSinRotation : public NEATSpatialAlgorithm {
    
    void stepSystem(ExtendedSystemInfo *individual);
    
    cpFloat evaluateSystem(ExtendedSystemInfo *sys);
    bool goodEnoughFitness(cpFloat bestFitness);
public:
    NEATSpatialSinRotation(int populationSize, int maxGenerations, int maxStagnation, float p_c=0.3, float p_m_attach=0.2, float p_m_node=0.1, float p_m_conn=0.1);
    
    virtual  char* inputDescription();
    virtual  char* outputDescription();
};

#endif /* defined(__SystemGenerator__NEATSpatialSinRotation__) */
