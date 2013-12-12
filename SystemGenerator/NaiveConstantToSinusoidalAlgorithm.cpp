//
//  NaiveConstantToSinusoidalAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/31/13.
//
//

#include "NaiveConstantToSinusoidalAlgorithm.h"
#include <numeric>
#include <algorithm>

NaiveConstantToSinusoidalAlgorithm::NaiveConstantToSinusoidalAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_m, float p_c)
:NaiveAlgorithm(maxGenerations, maxStagnation, p_m, p_c)
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

NaiveConstantToSinusoidalAlgorithm::~NaiveConstantToSinusoidalAlgorithm()
{
    for (int i=0; i<population.size(); i++)
        delete population[i];
}


MachineSystem * NaiveConstantToSinusoidalAlgorithm::createInitialSystem()
{
    MachineSystem *sys = new MachineSystem(300, 300, 6, 6, cpvzero);
    randomGenerator2(sys);
    return sys;
}


void NaiveConstantToSinusoidalAlgorithm::stepSystem(SystemInfo *individual)
{
    cpBody *inputBody = individual->system->partAtPosition(individual->system->inputMachinePosition)->body;
    cpBody *outputBody = individual->system->partAtPosition(individual->system->outputMachinePosition)->body;
    cpSpace *systemSpace = individual->system->getSpace();
    
    
    cpConstraint *motor = cpSimpleMotorNew(cpSpaceGetStaticBody(systemSpace), inputBody, M_PI);
    cpSpaceAddConstraint(systemSpace, motor);
    cpConstraintSetMaxForce(motor, 50000);
    
    for (int i=0; i<simSteps; i++) {
        individual->inputValues[i] = cpBodyGetAngle(inputBody);
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputValues[i] = cpBodyGetAngle(outputBody);
    }
    
    cpSpaceRemoveConstraint(systemSpace, motor);
   
}


static double angsin(double a)
{
    return 2*M_PI*sin(a);
}

// output rotation proportional to sin(input rotation)
cpFloat NaiveConstantToSinusoidalAlgorithm::evaluateSystem(SystemInfo *sys)
{
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputValues.size();
    
    std::vector<cpFloat>sinOfInput = std::vector<cpFloat>(nSteps);
    std::vector<cpFloat>normalizedOutput = std::vector<cpFloat>(nSteps);
    std::transform(sys->inputValues.begin(), sys->inputValues.end(), sinOfInput.begin(), angsin);
    
    std::transform(sys->outputValues.begin(), sys->outputValues.end(), normalizedOutput.begin(), normalize_angle);
    
    // now we assume sin(input) = c*output, and we try to minimize the error
    std::vector<cpFloat>sqDiff;
    
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    
    cpFloat inputMean = std::accumulate(sinOfInput.begin(), sinOfInput.end(), 0.0)/nSteps;
    
    cpFloat outputMean = std::accumulate(normalizedOutput.begin(), normalizedOutput.end(), 0.0)/nSteps;
        
    for (int i=0; i<nSteps; i++) {
        cpFloat diff = (sinOfInput[i] - normalizedOutput[i]);
        sqDiff.push_back(diff*diff);
        
        float xDiff = (sinOfInput[i]-inputMean);
        float yDiff = (normalizedOutput[i]-outputMean);
        sqrXDiffSum += xDiff*xDiff;
        sqrYDiffSum += yDiff*yDiff;
    }
    
    cpFloat inputVariance = sqrXDiffSum/nSteps;
    cpFloat outputVariance = sqrYDiffSum/nSteps;
    
    
    cpFloat mse = std::accumulate(sqDiff.begin(), sqDiff.end(), 0.0)/nSteps;
    
    fitness = inputVariance*outputVariance/mse;
    
    return fitness;

}

char* NaiveConstantToSinusoidalAlgorithm::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;
    
    snprintf(buffer, 100, "Input angle : %.3f ", cpBodyGetAngle(inputBody));
    
    return buffer;
}

char* NaiveConstantToSinusoidalAlgorithm::outputDescription()
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