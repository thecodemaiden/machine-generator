//
//  AdeolaConstantToSinusoidalAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/31/13.
//
//

#include "AdeolaConstantToSinusoidalAlgorithm.h"
#include <numeric>
#include <algorithm>

AdeolaConstantToSinusoidalAlgorithm::AdeolaConstantToSinusoidalAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_m, float p_c)
:AdeolaAlgorithm(maxGenerations, maxStagnation, p_m, p_c)
{
    simSteps = 40;
    population.resize(populationSize);
    
    bestIndividual = NULL;
    
    allTimeBestFitness = -FLT_MAX;
    
    for (int i=0; i<populationSize; i++) {
        population[i] = new SystemInfo(simSteps);
        population[i]->system = createInitialSystem();
    }
}

AdeolaConstantToSinusoidalAlgorithm::~AdeolaConstantToSinusoidalAlgorithm()
{
    for (int i=0; i<population.size(); i++)
        delete population[i];
}


MachineSystem *AdeolaConstantToSinusoidalAlgorithm::mutateSystem(MachineSystem *original)
{
    float selector = (float)rand()/RAND_MAX;
    
    if (selector > 0.75) {
        return  attachmentAnchorMutator(original);
    } else  if (selector > 0.5) {
        return  attachmentAnchorMutator2(original);
    } else if (selector > 0.25) {
        return addAttachmentMutator(original);
    } else {
        return outputMachineMutator(original);
    }
}

MachineSystem * AdeolaConstantToSinusoidalAlgorithm::createInitialSystem()
{
    MachineSystem *sys = new MachineSystem(300, 300, 6, 6, cpvzero);
    randomGenerator2(sys);
    return sys;
}

bool AdeolaConstantToSinusoidalAlgorithm::tick()
{
    size_t populationSize = population.size();
    
    cpFloat bestFitness = -INFINITY; // we want zero fitness
    cpFloat worstFitness = INFINITY;
    
    cpFloat lastBestFitness = bestIndividual ? bestIndividual->fitness : bestFitness;
    
    size_t bestPos = -1;
    size_t worstPos = -1;
    
    for (size_t popIter = 0; popIter <populationSize; popIter++) {
        // possibly mutate each one
        cpFloat random = (cpFloat)rand()/RAND_MAX;
        SystemInfo *individual = population[popIter];
        SystemInfo *mutant = NULL;
        if ( fabs(random) < p_m) {
            MachineSystem *system = individual->system;
            mutant = new SystemInfo(simSteps);
            mutant->system = mutateSystem(system);
        }
        
        // reset the individual... by copying!
        MachineSystem *original = individual->system;
        MachineSystem *copy = new MachineSystem(*original);
        individual->system = copy;
        delete original;
        
        stepSystem(individual);
        individual->fitness = evaluateSystem(individual);
        
        if (mutant) {
            stepSystem(mutant);
            mutant->fitness = evaluateSystem(mutant);
            
            if (mutant->fitness > individual->fitness) {
                population[popIter] = mutant;
                delete individual;
                individual = mutant;
            } else {
                delete mutant;
            }
        }
        
        if (individual->fitness > bestFitness) {
            bestFitness = individual->fitness;
            bestIndividual = individual;
            bestPos = popIter;
        }
        
        if (individual -> fitness <= worstFitness) {
            worstFitness = individual->fitness;
            worstPos = popIter;
        }
        
        
    }
    
    if (bestFitness > allTimeBestFitness)
        allTimeBestFitness = bestFitness;
    
    if (bestPos != worstPos) {
        // replace the worst one with a random machine
        
        // fprintf(stderr, "Replacing system with fitness %f\n", worstFitness);
        SystemInfo *worst = population[worstPos];
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
    
    bool stop =  (generations >= maxGenerations) || goodEnoughFitness(bestFitness) || (stagnantGenerations >= maxStagnation);
    
    generations++;
    if (stop) {
        fprintf(stderr, "ALL TIME BEST FITNESS: %f\n", allTimeBestFitness);
        
    }
    return  stop;
}


void AdeolaConstantToSinusoidalAlgorithm::stepSystem(SystemInfo *individual)
{
    cpBody *inputBody = individual->system->partAtPosition(individual->system->inputMachinePosition)->body;
    cpBody *outputBody = individual->system->partAtPosition(individual->system->outputMachinePosition)->body;
    cpSpace *systemSpace = individual->system->getSpace();
    
    
    cpConstraint *motor = cpSimpleMotorNew(cpSpaceGetStaticBody(systemSpace), inputBody, M_PI);
    cpSpaceAddConstraint(systemSpace, motor);
    cpConstraintSetMaxForce(motor, 50000);
    cpVect originalPosition = cpBodyGetPos(outputBody);
    
    for (int i=0; i<simSteps; i++) {
        individual->inputValues[i] = normalize_angle(cpBodyGetAngle(inputBody));
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputValues[i] = cpvdist(originalPosition, cpBodyGetPos(outputBody));
    }
    
    cpSpaceRemoveConstraint(systemSpace, motor);
   
}

bool compareMagnitude(const float &a, const float &b) {
    return fabs(a) < fabs(b);
}

cpFloat convertFromSin(cpFloat normalizer, cpFloat input) {
    if (normalizer == 0)
        normalizer = FLT_MIN;
    return asin(input/normalizer);
}

cpFloat AdeolaConstantToSinusoidalAlgorithm::evaluateSystem(SystemInfo *sys)
{
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputValues.size();
    
    cpFloat correlation = 0.0;
    
    
    cpFloat outputMax = *std::max_element(sys->outputValues.begin(), sys->outputValues.end());
    
    std::transform(sys->inputValues.begin(), sys->inputValues.end(), sys->inputValues.begin(), normalize_angle);
    cpFloat inputMean = std::accumulate(sys->inputValues.begin(), sys->inputValues.end(), 0.0)/nSteps;;
    
    
    for (int i=0; i<sys->outputValues.size(); i++)
        sys->outputValues[i] = convertFromSin(outputMax, sys->outputValues[i]);
    

   // std::transform(sys->outputValues.begin(), sys->outputValues.end(), sys->outputValues.begin(), sin);
    
    cpFloat outputMean = std::accumulate(sys->outputValues.begin(), sys->outputValues.end(), 0.0)/nSteps;;
    
  
    // correlation = sum[(x - mean(x))*(y - mean(y))] / sqrt(sum[(x - mean(x))^2] * sum[(y- mean(y))^2])
    
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
    //
    cpFloat inputVariance = sqrXDiffSum/nSteps;
    cpFloat outputVariance = sqrYDiffSum/nSteps;
    //
    if (correlation != correlation || fabs(correlation) == INFINITY) {
        // the mean output value, and all output values, were zero -> correlation = NaN
        // otherwise the mean input value, and all input values, were zero -> correlation = +/-inf
        correlation = 0;
    }
    
    //fitness = ;
    
    
    return fabs(correlation)*inputVariance*outputVariance; // 'ideal' fitness -> inf
}



bool AdeolaConstantToSinusoidalAlgorithm::goodEnoughFitness(cpFloat bestFitness)
{
    return bestFitness > 300;
}

MachineSystem * AdeolaConstantToSinusoidalAlgorithm::bestSystem()
{
    return bestIndividual->system;
}

char* AdeolaConstantToSinusoidalAlgorithm::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;
    
    snprintf(buffer, 100, "Input angle : %.3f ", cpBodyGetAngle(inputBody));
    
    return buffer;
}

char* AdeolaConstantToSinusoidalAlgorithm::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    
    static char buffer[100];
    MachinePart *bestMachine = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition);
    cpBody *outputBody = bestMachine->body;

    cpVect pos = cpBodyGetPos(outputBody);
    cpFloat disp = cpvdist(pos, bestMachine->getOriginalPosition());
    snprintf(buffer, 100, "Output displacement : %.3f ", disp);
    
    return buffer;
}