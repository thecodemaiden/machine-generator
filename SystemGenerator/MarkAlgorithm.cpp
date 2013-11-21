//
//  MarkAlgorithm.cpp
//  SystemGenerator
//
//  Created by Mark Whiting on 10/18/13.
//

#include "MarkAlgorithm.h"
#include <numeric>


// Take a state. Save the systems. Mutate or Crossover. Load the systems. Evaluate.

// Loading the state
// MachineSystem *s = MachineSystem::loadFromDisk("/Users/abannis/temp/test.txt");



//Make a population
MarkAlgorithm::MarkAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_m, float p_c):population(populationSize), p_m(p_m), maxStagnation(maxStagnation), p_c(p_c)
{
    for (int i=0; i<populationSize; i++) {
        population[i] = new MarkSystemInfo(simSteps);
        population[i]->system = createInitialSystem();
    }
    bestIndividual = NULL;
};

//Remove everyone from populations
MarkAlgorithm::~MarkAlgorithm(){
    // free the space
    for (int i=0; i<population.size(); i++) {
        delete population[i]->system;
        delete population[i];
    }
}

void MarkAlgorithm::stepSystem(MarkSystemInfo *individual)
{
    cpBody *inputBody = individual->system->partAtPosition(individual->system->inputMachinePosition)->body;
    cpBody *outputBody = individual->system->partAtPosition(individual->system->outputMachinePosition)->body;
    cpSpace *systemSpace = individual->system->getSpace();
    
    // set angvel of input body
    cpBodySetAngVel(inputBody, M_PI);
    
    for (int i=0; i<simSteps; i++) {
        individual->inputValues[i] = normalize_angle(cpBodyGetAngle(inputBody));
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputValues[i] = normalize_angle(cpBodyGetAngle(outputBody));
    }
}

//A counter
bool MarkAlgorithm::tick()
{
    size_t populationSize = population.size();
    
    float bestFitness = INFINITY; // we want zero fitness
    float worstFitness = -INFINITY;
    
    float lastBestFitness = bestIndividual ? bestIndividual->fitness : bestFitness;
    
    size_t bestPos = -1;
    size_t worstPos = -1;
    
    for (size_t popIter = 0; popIter <populationSize; popIter++) {
        // possibly mutate each one
        float random = (float)rand()/RAND_MAX;
        MarkSystemInfo *individual = population[popIter];
        MarkSystemInfo *mutant = NULL;
        if ( fabs(random) < p_m) {
            MachineSystem *system = individual->system;
            mutant = new MarkSystemInfo(simSteps);
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
        
        if (individual -> fitness >= worstFitness) {
            worstFitness = individual->fitness;
            worstPos = popIter;
        }
    }
    
    if (bestPos != worstPos) {
        // replace the worst one with a random machine
        
        // fprintf(stderr, "Replacing system with fitness %f\n", worstFitness);
        MarkSystemInfo *worst = population[worstPos];
        delete worst->system;
        worst->fitness = 0;
        worst->system = createInitialSystem();
    } else {
        fprintf(stderr, "Worst and best fitness are same - stop now?");
    }
    
    fprintf(stderr, "BEST FITNESS: %f (%lu)\n", bestFitness, bestPos);
    fprintf(stderr, "WORST FITNESS: %f (%lu)\n", worstFitness, worstPos);
    
    if (lastBestFitness == bestIndividual->fitness)
        stagnantGenerations++;
    else
        stagnantGenerations = 0;
    
    bool stop =  fabs(bestFitness) < 1e-7 && (bestFitness >= 0 || (stagnantGenerations >= maxStagnation));
    
    return  stop;
}

// (random) initializer
MachineSystem * MarkAlgorithm::createInitialSystem()
{
    MachineSystem *sys = new MachineSystem(300, 300, 5, 5, cpvzero);
    randomGenerator2(sys);
    return sys;
}

// operators
MachineSystem *MarkAlgorithm::mutateSystem(MachineSystem *original)
{
    // one or the other - twice the chance of mutator 1
    if (arc4random_uniform(3) == 0)
        return attachmentMutator2(original);
    else
        return attachmentMutator1(original);
}

cpFloat MarkAlgorithm::evaluateSystem(MarkSystemInfo *sys)
{
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputValues.size();
    
    cpFloat correlation = 0.0;
    
    cpFloat inputMean = std::accumulate(sys->inputValues.begin(), sys->inputValues.end(), 0.0)/nSteps;;
    cpFloat outputMean = std::accumulate(sys->outputValues.begin(), sys->outputValues.end(), 0.0)/nSteps;
    
    cpFloat numerator = 0.0;
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    
    for (int i=0; i<nSteps; i++) {
        float xDiff = (sys->inputValues[i]-inputMean);
        float yDiff = (sys->outputValues[i]-outputMean);
        numerator += xDiff*yDiff;
        sqrXDiffSum += xDiff*xDiff;
        sqrYDiffSum += yDiff*yDiff;
    }
    correlation = numerator/sqrt(sqrXDiffSum*sqrYDiffSum);
    
    cpFloat inputVariance = sqrXDiffSum/nSteps;
    cpFloat outputVariance = sqrYDiffSum/nSteps;
    
    if (correlation != correlation || fabs(correlation) == INFINITY) {
        // the mean output value, and all output values, were zero -> correlation = NaN
        // the mean input value, and all input values, were zero -> correlation = +/-inf
        fitness = 4.0;
    } else {
        fitness = 0.9*(1 - fabs(correlation));
        if (correlation > 0)
            fitness *= 0.01;
        if (inputVariance > 0 && outputVariance > 0)
            fitness /= inputVariance*outputVariance;
        else
            fitness *=4;
    }
    
    return fitness; // 'ideal' fitness -> 0
}

MachineSystem *MarkAlgorithm::bestSystem()
{
    return bestIndividual->system;
}

long MarkAlgorithm::getNumberOfIterations()
{
    return generations;
}

char* MarkAlgorithm::inputDescription()
{
    return "";
}

char* MarkAlgorithm::outputDescription()
{
    return "";
}

// generations = 0;
// a->inputDescription(), a->outputDescription());

//// almost certainly override
//virtual  char* inputDescription();
//virtual  char* outputDescription();
