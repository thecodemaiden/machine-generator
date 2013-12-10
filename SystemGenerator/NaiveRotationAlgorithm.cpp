//
//  NaiveBadAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/7/13.
//
//

#include "NaiveRotationAlgorithm.h"
#include <numeric>

NaiveRotationAlgorithm::NaiveRotationAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_m, float p_c)
:NaiveAlgorithm(maxGenerations, maxStagnation, p_m, p_c)
{
    simSteps = 100;
    population.resize(populationSize);
    
    bestIndividual = NULL;
    
    allTimeBestFitness = -FLT_MAX;
    
    for (int i=0; i<populationSize; i++) {
        population[i] = new SystemInfo(simSteps);
        population[i]->system = createInitialSystem();
    }
}

 NaiveRotationAlgorithm::~NaiveRotationAlgorithm(){
     for (int i=0; i<population.size(); i++)
         delete population[i];
 }

void NaiveRotationAlgorithm::stepSystem(SystemInfo *individual)
{
    
    MachineSystem *oldSystem = individual->system;
    individual->system = new MachineSystem(*oldSystem); // copy it to stop all the damn bouncing about
    delete oldSystem;
    
    cpBody *inputBody = individual->system->partAtPosition(individual->system->inputMachinePosition)->body;
    cpBody *outputBody = individual->system->partAtPosition(individual->system->outputMachinePosition)->body;
    cpSpace *systemSpace = individual->system->getSpace();

    
    cpConstraint *motor = cpSimpleMotorNew(cpSpaceGetStaticBody(systemSpace), inputBody, M_PI);
    cpSpaceAddConstraint(systemSpace, motor);
    cpConstraintSetMaxForce(motor, 50000);

    
    for (int i=0; i<simSteps; i++) {
        individual->inputValues[i] = (cpBodyGetAngle(inputBody));
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputValues[i] = (cpBodyGetAngle(outputBody));
    }
    cpSpaceRemoveConstraint(systemSpace, motor);
    cpBodySetAngVel(outputBody, 0);
    cpBodySetAngle(inputBody, 0);
}


// (random) initializer
MachineSystem * NaiveRotationAlgorithm::createInitialSystem()
{
    MachineSystem *sys = new MachineSystem(300, 300, 5, 5, cpvzero);
    randomGenerator2(sys);
    return sys;
}

// find correlation between input and ouput values
// if |correlation| -> 1, that's good (input is linearly related to output)
// I am looking for inputAngle = c*outputAngle, so I really want 1 (same sign)
// I also want the inputAngle to change a lot...

// let's try inputAngle:outputAngle = 2

cpFloat NaiveRotationAlgorithm::evaluateSystem(SystemInfo *sys)
{
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputValues.size();
    
    cpFloat correlation = 0.0;
    
    cpFloat inputMean = std::accumulate(sys->inputValues.begin(), sys->inputValues.end(), 0.0)/nSteps;;
    cpFloat outputMean = std::accumulate(sys->outputValues.begin(), sys->outputValues.end(), 0.0)/nSteps;
    
    // correlation = sum[(x - mean(x))*(y - mean(y))] / sqrt(sum[(x - mean(x))^2] * sum[(y- mean(y))^2])
    
    cpFloat numerator = 0.0;
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    
    cpFloat meandInputdOutput = 0.0;
    
    for (int i=0; i<nSteps; i++) {
        float xDiff = (sys->inputValues[i]-inputMean);
        float yDiff = (sys->outputValues[i]-outputMean);
        numerator += xDiff*yDiff;
        sqrXDiffSum += xDiff*xDiff;
        sqrYDiffSum += yDiff*yDiff;
        
        if (i>0)
            meandInputdOutput += (sys->inputValues[i] - sys->inputValues[i-1])/(sys->outputValues[i]-sys->outputValues[i-1]);
    }
    correlation = numerator/sqrt(sqrXDiffSum*sqrYDiffSum);
    
   cpFloat inputVariance = sqrXDiffSum/nSteps;
    cpFloat outputVariance = sqrYDiffSum/nSteps;
    
    if (isUnreasonable(correlation)) {
        // input and output were zero -> correlation = NaN
        // otherwise the mean input value, and all input values, were zero -> correlation = +/-inf
        correlation = 0;
    }
    
    if (isUnreasonable(meandInputdOutput)) {
        meandInputdOutput = 0;
    } else {
        meandInputdOutput /= nSteps;
    }
    
    fitness = fabs(correlation) + 0.5*correlation;
    
    if (fitness == 0 || inputVariance < 5e-1 || outputVariance < 5e-1) {
        fitness = 1e-8;
    } else {
        cpFloat distance = fabs (2.0 -meandInputdOutput);
        fitness /= (distance+0.1)*(distance+0.1);
    }
    
    assert(!isUnreasonable(fitness));
    
    return fitness;
}


 char* NaiveRotationAlgorithm::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;

    snprintf(buffer, 100, "Input angle : %.3f", cpBodyGetAngle(inputBody));
    
    return buffer;
}

 char* NaiveRotationAlgorithm::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *outputBody = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition)->body;
    
    snprintf(buffer, 100, "Output angle : %.3f", cpBodyGetAngle(outputBody));
    
    return buffer;
}
