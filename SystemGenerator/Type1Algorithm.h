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


typedef void (*spaceUpdateFunc)(cpSpace *space, long steps, cpFloat stepTime, bool visible, cpVect translation);

// this is an abstract class - there is no code supplied for most of these methods
class Type1Algorithm {

public:
    
    Type1Algorithm(){};
    virtual ~Type1Algorithm(){};
    
    // perform parent selection, offspring production, fitness evaluation, and re-population
    void tick();
    
    // set this for use in tick() - steps the simulation
    spaceUpdateFunc updateFunction;
    
private:
    // (random) initializer
    virtual MachineSystem *createInitialSystem() = 0;
    
    // operators
    virtual MachineSystem *mutateSystem(MachineSystem *original) = 0;
    virtual void combineSystems(MachineSystem *parent1, MachineSystem *parent2, MachineSystem **child1, MachineSystem **child2) = 0;
    
    // write your own fitness evaluator
    
    // the main loop will keep state updated with this 'post-step callback'
    // 'key' allows you to determine which object this was called for
    // 'data' is currently unused
    virtual void afterWorldStep(cpSpace *space, void *key, void *data) = 0;
};

// example operators/generators
void randomGenerator1(MachineSystem *sys);
MachineSystem *attachmentMutator1(MachineSystem *sys);



#endif /* defined(__SystemGenerator__Type1Algorithm__) */
