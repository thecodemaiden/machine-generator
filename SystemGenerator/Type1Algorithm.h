//
//  Type1Algorithm.h
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/6/13.
//
//

#ifndef __SystemGenerator__Type1Algorithm__
#define __SystemGenerator__Type1Algorithm__

#include "MachineSystem.h"
#include <vector>

// this is an 'abstract class' - there is no code supplied for most of these methods
class Type1Algorithm {
private:
    std::vector<MachineSystem *> population;
    std::vector<float> fitnesses;
public:
    
    // basic implementation of constructor and destructor for subclasses to build on
    Type1Algorithm(int populationSize = 100)
     : population(std::vector<MachineSystem *>(populationSize)),
       fitnesses(std::vector<float>(populationSize))
    {
        // each system should probably get its own space
    }
    
    virtual ~Type1Algorithm(){
    
    };
    
    // (random) initializer
    virtual void populateSystem(MachineSystem *sys) = 0;
    
    // operators
    virtual MachineSystem *mutateSystem(MachineSystem *original) = 0;
    virtual void combineSystems(MachineSystem *parent1, MachineSystem *parent2, MachineSystem **child1, MachineSystem **child2) = 0;
    
    // fitness evaluator
    virtual cpFloat *evaluateSystem(MachineSystem *sys) = 0;
    
    // the main loop will keep state updated with this 'post-step callback'
    // 'key' should be the address of this object
    // 'data' is currently NULL
    cpPostStepFunc afterWorldStep(cpSpace *space, void *key, void *data);
};

// a bunch of operators/generators that can be combined in Type1Algorithm subclasses

void randomGenerator1(MachineSystem *sys);



#endif /* defined(__SystemGenerator__Type1Algorithm__) */
