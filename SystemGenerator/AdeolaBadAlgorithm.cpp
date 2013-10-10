//
//  AdeolaBadAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/7/13.
//
//

#include "AdeolaBadAlgorithm.h"


// basic implementation of constructor and destructor for subclasses to build on
AdeolaAlgorithm::AdeolaAlgorithm(int populationSize, float p_m)
:population(populationSize),
 p_m(p_m)
{
    for (int i=0; i<populationSize; i++) {
        population[i] = new SystemInfo(simSteps);
        population[i]->space = cpSpaceNew();
        population[i]->system = createInitialSystem(population[i]->space);
    }
    bestIndividual = NULL;
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
    size_t populationSize = population.size();
    
    float maxFitness = -FLT_MAX;
    size_t bestPos = -1;
    for (size_t popIter = 0; popIter <populationSize; popIter++) {
        // mutate each one
    
        float random = (float)rand()/RAND_MAX;
        SystemInfo *individual = population[popIter];
        if (random < p_m) {
            MachineSystem *system = individual->system;
            individual->system = mutateSystem(system);
            delete system;
        }
        
        // set angvel of input body
        cpBody *inputBody = individual->system->partAtPosition(individual->system->inputMachinePosition)->body;
        cpBody *outputBody = individual->system->partAtPosition(individual->system->outputMachinePosition)->body;
        cpBodySetAngVel(inputBody, M_PI_4);
        
        for (int i=0; i<simSteps; i++) {
            individual->inputValues[i] = cpBodyGetAngle(inputBody);
            cpSpaceStep(individual->space, 0.5);
            individual->outputValues[i] = cpBodyGetAngle(outputBody);
        }
        
        cpBodySetAngVel(inputBody, 0);
        cpBodyResetForces(outputBody);
        
        individual->fitness = evaluateSystem(individual);
        if (individual->fitness > maxFitness) {
            maxFitness = individual->fitness;
            bestIndividual = individual;
            bestPos = popIter;
        }
    }
    
    fprintf(stderr, "BEST FITNESS: %f (%lu)\n", maxFitness, bestPos);
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
    return attachmentMutator1(original);
}

// fitness evaluator
cpFloat AdeolaAlgorithm::evaluateSystem(SystemInfo *sys)
{
    cpFloat fitness = 0.0;
    size_t nSteps = sys->inputValues.size();
    
    for (size_t i=0; i<nSteps; i++) {
        float error = sys->inputValues[i] - sys->outputValues[i];
        fitness += error*error;
    }
    
    return -fitness; // we want zero error
}

cpSpace * AdeolaAlgorithm::simulationSpaceForBestIndividual()
{
    return bestIndividual->space;
}

MachineSystem *AdeolaAlgorithm::bestSystem()
{
    return bestIndividual->system;
}