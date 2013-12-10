//
//  NaiveDisplacementAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/14/13.
//
//

#include "NaiveDisplacementAlgorithm.h"
#include <numeric>

NaiveDisplacementAlgorithm::NaiveDisplacementAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_m, float p_c)
:NaiveAlgorithm(maxGenerations, maxStagnation, p_m, p_c)
{
    simSteps = 30;
    population.resize(populationSize);
    
    bestIndividual = NULL;
    
    for (int i=0; i<populationSize; i++) {
        population[i] = new SystemInfo(simSteps);
        population[i]->system = createInitialSystem();
    }
}


NaiveDisplacementAlgorithm::~NaiveDisplacementAlgorithm(){
    for (int i=0; i<population.size(); i++)
        delete population[i];
}

bool getSystemToRest(MachineSystem *sys)
{
    cpSpace *space = sys->getSpace();
    
    bool atRest = false;
    const cpFloat maxTime = 10.0;
    __block cpFloat kineticEnergy;
    
    for (cpFloat timeLeft = maxTime; timeLeft > 0; timeLeft-=0.1) {
        cpSpaceStep(space, 0.2);
        kineticEnergy = 0.0;
        cpSpaceEachBody_b(space, ^(cpBody *body) {
            cpFloat v2 = cpvlengthsq(cpBodyGetVel(body));
            cpFloat w2 = (cpBodyGetAngVel(body));
            w2 *= w2;
            kineticEnergy += 0.5*cpBodyGetMass(body)*v2;
            kineticEnergy += 0.5*cpBodyGetMoment(body)*w2;
        });
        if (fabs(kineticEnergy) < 5){
            atRest = true;
            break;
        }
        
    }
    return atRest;
}

void NaiveDisplacementAlgorithm::stepSystem(SystemInfo *individual)
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
   
        // destroy and rebuild system - i haven't figured out another way to reset all forces, velocities and positions....
            MachineSystem *systemCopy = new MachineSystem(*individual->system);
            delete individual->system;
            individual->system = systemCopy;
    
}


// (random) initializer
MachineSystem * NaiveDisplacementAlgorithm::createInitialSystem()
{
    MachineSystem *sys = new MachineSystem(300, 300, 5, 5, cpvzero);
    randomGenerator3(sys);
    return sys;
}


// I want input angle to change a lot,
// and I want the max displacement as big as possible

cpFloat NaiveDisplacementAlgorithm::evaluateSystem(SystemInfo *sys)
{
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputValues.size();
    
    
    cpFloat inputMean = std::accumulate(sys->inputValues.begin(), sys->inputValues.end(), 0.0)/nSteps;;
    cpFloat outputMean = std::accumulate(sys->outputValues.begin(), sys->outputValues.end(), 0.0)/nSteps;
    
    
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    cpFloat maxDisplacement = 0.0;
    
    for (int i=0; i<nSteps; i++) {
        float xDiff = (sys->inputValues[i]-inputMean);
        float yDiff = (sys->outputValues[i]-outputMean);
        
        maxDisplacement = MAX(sys->outputValues[i], maxDisplacement);
        
        sqrXDiffSum += xDiff*xDiff;
        sqrYDiffSum += yDiff*yDiff;
    }
    
    cpFloat inputVariance = sqrXDiffSum/nSteps;
    cpFloat outputVariance = sqrYDiffSum/nSteps;
    
    fitness = (maxDisplacement+inputVariance)*outputVariance;
    
    return fitness; // 'ideal' fitness -> infinity
}


char* NaiveDisplacementAlgorithm::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;
    
    snprintf(buffer, 100, "Input angle : %.3f ", cpBodyGetAngle(inputBody));
    
    return buffer;
}

char* NaiveDisplacementAlgorithm::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *outputBody = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition)->body;
    
    cpVect pos = cpBodyGetPos(outputBody);
    snprintf(buffer, 100, "Output position : (%.3f, %.3f) ", pos.x, pos.y);
    
    return buffer;
}
