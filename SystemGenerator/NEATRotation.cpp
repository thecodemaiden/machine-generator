//
//  NEATRotation.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/4/13.
//
//

#include "NEATRotation.h"
#include <numeric>

NEATRotation::NEATRotation(int populationSize, int maxGenerations, int maxStagnation, float p_c, float p_m_attach, float p_m_node, float p_m_conn)
:NEATAlgorithm(populationSize, maxGenerations, maxStagnation, p_c, p_m_attach, p_m_node, p_m_conn)
{ 
    w_excess = 1.0;
    w_disjoint = 1.0;
    w_matching = 0.5;
    insertRandomAttachments = false;
    simSteps = 300;
}

void NEATRotation::stepSystem(ExtendedSystemInfo *individual)
{
    MachineSystem *sys = new MachineSystem(*individual->system);
    delete individual->system;
    individual->system = sys;
    
    cpBody *inputBody = sys->partAtPosition(individual->system->inputMachinePosition)->body;
    cpBody *outputBody = sys->partAtPosition(individual->system->outputMachinePosition)->body;
    cpSpace *systemSpace = sys->getSpace();
    
    cpConstraint *motor = cpSimpleMotorNew(cpSpaceGetStaticBody(systemSpace), inputBody, M_PI);
    cpSpaceAddConstraint(systemSpace, motor);
    cpConstraintSetMaxForce(motor, 50000);
    
    for (int i=0; i<simSteps; i++) {
        individual->inputRotationAngle[i] = (cpBodyGetAngle(inputBody));
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputRotationAngle[i] = (cpBodyGetAngle(outputBody));
    }
    
    cpSpaceRemoveConstraint(systemSpace, motor);
}

static bool isUnreasonable(double n) {
    return fabs(n) == INFINITY || n != n;
}

cpFloat NEATRotation::evaluateSystem(ExtendedSystemInfo *sys)
{
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputRotationAngle.size();
    
    cpFloat correlation = 0.0;
    
    cpFloat inputMean = std::accumulate(sys->inputRotationAngle.begin(), sys->inputRotationAngle.end(), 0.0)/nSteps;;
    cpFloat outputMean = std::accumulate(sys->outputRotationAngle.begin(), sys->outputRotationAngle.end(), 0.0)/nSteps;
    
    // correlation = sum[(x - mean(x))*(y - mean(y))] / sqrt(sum[(x - mean(x))^2] * sum[(y- mean(y))^2])
    
    cpFloat numerator = 0.0;
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    
    cpFloat meandInputdOutput = 0.0;
    
    for (int i=0; i<nSteps; i++) {
        float xDiff = (sys->inputRotationAngle[i]-inputMean);
        float yDiff = (sys->outputRotationAngle[i]-outputMean);
        numerator += xDiff*yDiff;
        sqrXDiffSum += xDiff*xDiff;
        sqrYDiffSum += yDiff*yDiff;
        
        if (i>0)
            meandInputdOutput += (sys->inputRotationAngle[i] - sys->inputRotationAngle[i-1])/(sys->outputRotationAngle[i]-sys->outputRotationAngle[i-1]);
    }
    correlation = numerator/sqrt(sqrXDiffSum*sqrYDiffSum);
    //
    cpFloat inputVariance = sqrXDiffSum/nSteps;
    cpFloat outputVariance = sqrYDiffSum/nSteps;
    //
    if (isUnreasonable(correlation)) {
        // the mean output value, and all output values, were zero -> correlation = NaN
        // otherwise the mean input value, and all input values, were zero -> correlation = +/-inf
        correlation = 0;
    }
    
    if (isUnreasonable(meandInputdOutput)) {
        meandInputdOutput = 0;
    } else {
        meandInputdOutput /= nSteps;
    }
    
    fitness = 1.0+correlation;
    
    if (outputVariance > 0) {
        cpFloat distance = fabs (2.0 -meandInputdOutput);
        fitness /= (distance+0.1)*(distance+0.1);
    }

    if (fitness == 0 || inputVariance < 0.05 || outputVariance < 0.05 || isUnreasonable(inputVariance) || isUnreasonable(outputVariance))
        fitness = 1e-8;

    
    assert(fitness == fitness);
    assert(fabs(fitness) != INFINITY);
    
    return fitness;
}

bool NEATRotation::goodEnoughFitness(cpFloat bestFitness)
{
    return false;
}

char* NEATRotation::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;
    
    snprintf(buffer, 100, "Input angle : %.3f",  cpBodyGetAngle(inputBody));
    
    return buffer;
}

char* NEATRotation::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *outputBody = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition)->body;
    
    snprintf(buffer, 100, "Output angle : %.3f", cpBodyGetAngle(outputBody));
    
    return buffer;
}

void NEATRotation::prepareInitialPopulation()
{
    MachineSystem *initialSystem = new MachineSystem(300, 300, 5, 5, cpvzero);
    neatGenerator(initialSystem);
    
    // create one of each attachment type
    for (int i=0; i< ATTACH_TYPE_MAX; i++) {
        MachineSystem *newSys = new MachineSystem(*initialSystem);
        Attachment *newAtt = Attachment::createAttachmentOfType((AttachmentType)i);
        newSys->updateAttachmentBetween(newSys->inputMachinePosition, newSys->outputMachinePosition, newAtt);
        ExtendedSystemInfo *s = new ExtendedSystemInfo(simSteps);
        s->system = newSys;
        population.push_back(s);
    }
    delete initialSystem;
}