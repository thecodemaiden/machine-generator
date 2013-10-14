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
    cpSpace *systemSpace = individual->system->getSpace();

    // set angvel of input body
    cpBodySetAngVel(inputBody, M_PI);
    
    for (int i=0; i<simSteps; i++) {
        individual->inputValues[i] = normalize_angle(cpBodyGetAngle(inputBody));
       // cpBodySetAngle(inputBody, angV);
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputValues[i] = normalize_angle(cpBodyGetAngle(outputBody));
    }
    
//           cpBodySetAngVel(inputBody, 0);
//            cpBodyResetForces(outputBody);
}

void AdeolaAlgorithm::tick()
{
    size_t populationSize = population.size();
    
    float bestFitness = FLT_MAX; // we want zero fitness
    float worstFitness = -FLT_MAX;
    
    size_t bestPos = -1;
    size_t worstPos = -1;
    
    for (size_t popIter = 0; popIter <populationSize; popIter++) {
        // possibly mutate each one
        float random = (float)rand()/RAND_MAX;
        SystemInfo *individual = population[popIter];
        SystemInfo *mutant = NULL;
        if ( fabs(random) < p_m) {
            MachineSystem *system = individual->system;
            mutant = new SystemInfo(simSteps);
            mutant->system = mutateSystem(system);
        }
        
        
        stepSystem(individual);
        individual->fitness = evaluateSystem(individual);

        if (mutant) {
            stepSystem(mutant);
            mutant->fitness = evaluateSystem(mutant);
            
            if (mutant->fitness < individual->fitness) {
                population[popIter] = mutant;
                delete individual;
                individual = mutant;
                //fprintf(stderr, "Mutation successful. (%.3f->%.3f)\n", individual->fitness, mutant->fitness);
            } else {
                delete mutant;
            }
        }

        
        if (individual->fitness < bestFitness) {
            bestFitness = individual->fitness;
            bestIndividual = individual;
            bestPos = popIter;
        }
        
        if (individual -> fitness > worstFitness) {
            worstFitness = individual->fitness;
            worstPos = popIter;
        }
    }
    
    if (bestPos != worstPos) {
        // replace the worst one with a random machine
        
        fprintf(stderr, "Replacing system with fitness %f\n", worstFitness);
        SystemInfo *worst = population[worstPos];
        delete worst->system;
        worst->fitness = 0;
        worst->system = createInitialSystem();
    } else {
        fprintf(stderr, "Worst and best fitness are same - stop now?");
    }
    
   // fprintf(stderr, "BEST FITNESS: %f (%lu)\n", bestFitness, bestPos);
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
    // one or the other - twice the chance of mutator 1
    if (arc4random_uniform(3) == 0)
        return attachmentMutator2(original);
    else
        return attachmentMutator1(original);
}

// fitness evaluator
cpFloat AdeolaAlgorithm::evaluateSystem(SystemInfo *sys)
{
    cpFloat fitness = 0.0;
    size_t nSteps = sys->inputValues.size();
    
    for (size_t i=0; i<nSteps; i++) {
        float error = fabs(sys->inputValues[i] - sys->outputValues[i]);
        if (error == 0) error = 1e-20; // we don't want an infinite fitness because one error was 0
        fitness += 2*log(error); // these errors are very small at times
    }
    
    return fitness; // 'ideal' fitness -> -inf
}

MachineSystem *AdeolaAlgorithm::bestSystem()
{
    return bestIndividual->system;
}