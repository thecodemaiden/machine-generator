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
        population[i]->system = createInitialSystem();
    }
    bestIndividual = NULL;
};

 AdeolaAlgorithm::~AdeolaAlgorithm(){
     // free the space
     for (int i=0; i<population.size(); i++) {
         delete population[i]->system;
         delete population[i];
     }
 }

cpFloat normalize_angle(cpFloat angle)
{
    return fmodf(angle, 2*M_PI);
}

void AdeolaAlgorithm::stepSystem(SystemInfo *individual)
{
    cpBody *inputBody = individual->system->partAtPosition(individual->system->inputMachinePosition)->body;
    cpBody *outputBody = individual->system->partAtPosition(individual->system->outputMachinePosition)->body;
    
    // set angvel of input body
    cpBodySetAngVel(inputBody, M_PI/4);
    
    for (int i=0; i<simSteps; i++) {
        cpFloat ang = cpBodyGetAngle(inputBody) +  M_PI/20;
        individual->inputValues[i] = normalize_angle(ang);
       // cpBodySetAngle(inputBody, angV);
        cpSpace *systemSpace = individual->system->getSpace();
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputValues[i] = normalize_angle(cpBodyGetAngle(outputBody));
    }
    
//           cpBodySetAngVel(inputBody, 0);
//            cpBodyResetForces(outputBody);
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
        SystemInfo *mutant = NULL;
        if ( fabs(random) < p_m) {
            MachineSystem *system = individual->system;
            mutant = new SystemInfo(simSteps);
            mutant->system = new MachineSystem(*system);
        }
        
        
        stepSystem(individual);
        individual->fitness = evaluateSystem(individual);

        if (mutant) {
            float oldFitness = individual->fitness;
            
            stepSystem(mutant);
            mutant->fitness = evaluateSystem(mutant);
            
            float newFitness = mutant->fitness;
            
            if (mutant->fitness > individual->fitness) {
                population[popIter] = mutant;
                delete individual;
                individual = mutant;
                fprintf(stderr, "Mutation successful. (%.3f->%.3f)\n", oldFitness, newFitness);
            }
        }

        
        if (individual->fitness > maxFitness) {
            maxFitness = individual->fitness;
            bestIndividual = individual;
            bestPos = popIter;
        }
    }
    
   // fprintf(stderr, "BEST FITNESS: %f (%lu)\n", maxFitness, bestPos);
}

// (random) initializer
MachineSystem * AdeolaAlgorithm::createInitialSystem()
{
    MachineSystem *sys = new MachineSystem(300, 300, 5, 5, cpvzero);
    randomGenerator2(sys);
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

MachineSystem *AdeolaAlgorithm::bestSystem()
{
    return bestIndividual->system;
}