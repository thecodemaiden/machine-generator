//
//  NaiveDisplacementAlgorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/14/13.
//
//

#ifndef __SystemGenerator__NaiveDisplacementAlgorithm__
#define __SystemGenerator__NaiveDisplacementAlgorithm__

#include "NaiveBaseAlgorithm.h"

// ok harder - I am going to rotate the input and I want the output to move, not rotate
// going to try recombination as well

class NaiveDisplacementAlgorithm : public NaiveAlgorithm {
    MachineSystem *createInitialSystem();
    void stepSystem(SystemInfo *individual);
    
    cpFloat evaluateSystem(SystemInfo *sys);
    
public:
    
    NaiveDisplacementAlgorithm(int populationSize=5, int maxGenerations = 100, int maxStagnation=5, float p_m=0.02, float p_c= 0);
    ~NaiveDisplacementAlgorithm();

    
    char* inputDescription();
    char* outputDescription();

};

#endif /* defined(__SystemGenerator__NaiveDisplacementAlgorithm__) */
