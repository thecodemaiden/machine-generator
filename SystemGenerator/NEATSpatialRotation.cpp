//
//  NEATSpatialRotation.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/15/13.
//
//

#include "NEATSpatialRotation.h"

#include <numeric>

NEATSpatialRotation::NEATSpatialRotation(int populationSize, int maxGenerations, int maxStagnation, float p_c, float p_m_attach, float p_m_node, float p_m_conn)
: NEATSpatialAlgorithm(populationSize, maxGenerations, maxStagnation, p_c, p_m_attach, p_m_node, p_m_conn)
{
    w_excess = 1.0;
    w_disjoint = 1.0;
    w_matching = 0.5;
    insertRandomAttachments = true;
    simSteps = 250;
}

void NEATSpatialRotation::stepSystem(ExtendedSystemInfo *individual)
{
    MachineSystem *sys = individual->system;
    
    cpBody *inputBody = sys->partAtPosition(sys->inputMachinePosition)->body;
    cpBody *outputBody = sys->partAtPosition(sys->outputMachinePosition)->body;
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

cpFloat NEATSpatialRotation::evaluateSystem(ExtendedSystemInfo *sys)
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
    if (correlation != correlation || fabs(correlation) == INFINITY) {
        // the mean output value, and all output values, were zero -> correlation = NaN
        // otherwise the mean input value, and all input values, were zero -> correlation = +/-inf
        correlation = 0;
    }
    
    if (meandInputdOutput != meandInputdOutput || fabs(meandInputdOutput) == INFINITY) {
        meandInputdOutput = 0;
    } else {
        meandInputdOutput /= nSteps;
    }
    
    fitness = 1.0+correlation;
    
    if (outputVariance > 0) {
        cpFloat distance = fabs (2.0 -meandInputdOutput);
        fitness /= (distance+0.1)*(distance+0.1);
    }
    
    if (fitness == 0 || inputVariance < 5e-1 || outputVariance < 5e-1)
        fitness = 1e-8;
    
    
    
    assert(fitness == fitness);
    assert(fabs(fitness) != INFINITY);
    
    return fitness;
}

bool NEATSpatialRotation::goodEnoughFitness(cpFloat bestFitness)
{
    return false;
}

char* NEATSpatialRotation::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;
    
    snprintf(buffer, 100, "Input angle : %.3f", normalize_angle(cpBodyGetAngle(inputBody)));
    
    return buffer;
}

char* NEATSpatialRotation::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *outputBody = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition)->body;
    
    snprintf(buffer, 100, "Output angle : %.3f", normalize_angle(cpBodyGetAngle(outputBody)));
    
    return buffer;
}

