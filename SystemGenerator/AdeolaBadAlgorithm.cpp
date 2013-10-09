//
//  AdeolaBadAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/7/13.
//
//

#include "AdeolaBadAlgorithm.h"


// basic implementation of constructor and destructor for subclasses to build on
AdeolaAlgorithm::AdeolaAlgorithm(int populationSize)
:Type1Algorithm(),
  population(populationSize),
   lastInputValues(stepSize)
{
    for (int i=0; i<populationSize; i++) {
        population[i] = new SystemInfo(stepSize);
        population[i]->space = cpSpaceNew();
        population[i]->system = createInitialSystem(population[i]->space);
    }
};

 AdeolaAlgorithm::~AdeolaAlgorithm(){
     // free the space
     for (int i=0; i<population.size(); i++) {
         cpSpaceFree(population[i]->space);
         delete population[i]->system;
         delete population[i];
     }
 }

void AdeolaAlgorithm::tick()
{
    
}

// (random) initializer
MachineSystem * AdeolaAlgorithm::createInitialSystem(cpSpace *space)
{
    MachineSystem *sys = new MachineSystem(300, 300, 10, 10, cpvzero, space);
    randomGenerator1(sys);
    return sys;
}

// operators
MachineSystem *AdeolaAlgorithm::mutateSystem(MachineSystem *original)
{
    float random = rand()/RAND_MAX;
    if (random < p_m)
        return attachmentMutator1(original);
    return NULL;
}

void AdeolaAlgorithm::combineSystems(MachineSystem *parent1, MachineSystem *parent2, MachineSystem **child1, MachineSystem **child2)
{
    // no recombination yet
    if (child1)
        *child1 = NULL;
    if (child2)
        *child2 = NULL;
}

// fitness evaluator
cpFloat AdeolaAlgorithm::evaluateSystem(MachineSystem *sys)
{
    return 0.0;
}

// the main loop will keep state updated with this 'post-step callback'
// 'key' should be the address of this object
// 'data' is currently NULL
void AdeolaAlgorithm::afterWorldStep(cpSpace *space, void *key, void *data)
{
    
}