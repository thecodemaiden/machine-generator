//
//  AdeolaDisplacementAlgorithm.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 10/14/13.
//
//

#include "AdeolaDisplacementAlgorithm.h"
#include <numeric>

AdeolaDisplacementAlgorithm::AdeolaDisplacementAlgorithm(int populationSize, int maxGenerations, int maxStagnation, float p_m, float p_c)
:AdeolaAlgorithm(maxGenerations, maxStagnation, p_m, p_c)
{
    simSteps = 30;
    population.resize(populationSize);
    
    bestIndividual = NULL;
    
    for (int i=0; i<populationSize; i++) {
        population[i] = new SystemInfo(simSteps);
        population[i]->system = createInitialSystem();
    }
}


AdeolaDisplacementAlgorithm::~AdeolaDisplacementAlgorithm(){
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


bool AdeolaDisplacementAlgorithm::tick()
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
        
        if (!getSystemToRest(individual->system)) {
            individual->fitness = -1000;
        } else {
            SystemInfo *mutant = NULL;
            if ( fabs(random) < p_m) {
                MachineSystem *system = individual->system;
                mutant = new SystemInfo(simSteps);
                mutant->system = mutateSystem(system);
            }
            
            stepSystem(individual);
            individual->fitness = evaluateSystem(individual);
            
            if (mutant) {
                if (getSystemToRest(mutant->system)) {
                    stepSystem(mutant);
                    mutant->fitness = evaluateSystem(mutant);
                } else {
                    mutant->fitness = -1000;
                }
                
                if (mutant->fitness > individual->fitness) {
                    population[popIter] = mutant;
                    delete individual;
                    individual = mutant;
                } else {
                    delete mutant;
                }
                
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
    
    return  stop;
}




void AdeolaDisplacementAlgorithm::stepSystem(SystemInfo *individual)
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
MachineSystem * AdeolaDisplacementAlgorithm::createInitialSystem()
{
    MachineSystem *sys = new MachineSystem(300, 300, 5, 5, cpvzero);
    randomGenerator3(sys);
    return sys;
}

// operators
MachineSystem *AdeolaDisplacementAlgorithm::mutateSystem(MachineSystem *original)
{
    // one or the other - twice the chance of mutator 1
    if (arc4random_uniform(3) == 0)
        return attachmentMutator2(original);
    else
        return attachmentMutator1(original);
}


// I want input angle to change a lot,
// and I want the max displacement as big as possible

cpFloat AdeolaDisplacementAlgorithm::evaluateSystem(SystemInfo *sys)
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

bool AdeolaDisplacementAlgorithm::goodEnoughFitness(cpFloat bestFitness)
{
    return false; // never satisfied
}

MachineSystem *AdeolaDisplacementAlgorithm::bestSystem()
{
    return bestIndividual->system;
}

char* AdeolaDisplacementAlgorithm::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;
    
    snprintf(buffer, 100, "Input angle : %.3f ", cpBodyGetAngle(inputBody));
    
    return buffer;
}

char* AdeolaDisplacementAlgorithm::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *outputBody = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition)->body;
    
    cpVect pos = cpBodyGetPos(outputBody);
    snprintf(buffer, 100, "Output position : (%.3f, %.3f) ", pos.x, pos.y);
    
    return buffer;
}
