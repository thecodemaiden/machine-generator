//
//  AdeolaConstantToSinusoidalAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/31/13.
//
//

#ifndef __SystemGenerator__AdeolaConstantToSinusoidalAlgorithm__
#define __SystemGenerator__AdeolaConstantToSinusoidalAlgorithm__

#include "AdeolaRotationAlgorithm.h"

class AdeolaConstantToSinusoidalAlgorithm : public AdeolaRotationAlgorithm {
    
public:
    AdeolaConstantToSinusoidalAlgorithm(int populationSize=5, int maxGenerations = 100, int maxStagnation=5, float p_m=0.2, float p_c= 0)
    :AdeolaRotationAlgorithm(populationSize, maxGenerations, maxStagnation, p_m, p_c){};

    // the only real difference is how I evaluate the output - now looking for input = a sin(output + phase) + b
    cpFloat evaluateSystem(SystemInfo *sys);
    bool goodEnoughFitness(cpFloat bestFitness);
    
};

#endif