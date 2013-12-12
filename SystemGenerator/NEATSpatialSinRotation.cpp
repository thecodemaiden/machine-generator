//
//  NEATSpatialSinRotation.cpp
//  SystemGenerator
//
//  Created by Adeola Bannis on 11/25/13.
//
//

#include "NEATSpatialSinRotation.h"
#include <numeric>

NEATSpatialSinRotation::NEATSpatialSinRotation(int populationSize, int maxGenerations, int maxStagnation, float p_c, float p_m_attach, float p_m_node, float p_m_conn)
: NEATSpatialAlgorithm(populationSize, maxGenerations, maxStagnation, p_c, p_m_attach, p_m_node, p_m_conn)
{
    d_threshold = 3.0;
    w_excess = 2.0;
    w_disjoint = 1.0;
    w_matching = 3.0;
    insertRandomAttachments = false;
    simSteps = 250;
}

void NEATSpatialSinRotation::stepSystem(ExtendedSystemInfo *individual)
{

    MachineSystem *oldSystem = individual->system;
    individual->system = new MachineSystem(*oldSystem); // copy it to stop all the damn bouncing about
    delete oldSystem;
    
    MachineSystem *sys = individual->system;
    
    cpBody *inputBody = sys->partAtPosition(sys->inputMachinePosition)->body;
    cpBody *outputBody = sys->partAtPosition(sys->outputMachinePosition)->body;
    cpSpace *systemSpace = sys->getSpace();
    
    cpConstraint *motor = cpSimpleMotorNew(cpSpaceGetStaticBody(systemSpace), inputBody, M_PI/6);
    cpSpaceAddConstraint(systemSpace, motor);
    cpConstraintSetMaxForce(motor, 50000);
    
    for (int i=0; i<simSteps; i++) {
        individual->inputRotationAngle[i] = (cpBodyGetAngle(inputBody));
        
        cpSpaceStep(systemSpace, 0.1);
        individual->outputRotationAngle[i] = (cpBodyGetAngle(outputBody));
    }
    
    cpSpaceRemoveConstraint(systemSpace, motor);
    
}

static double angsin(double a)
{
    return 2*M_PI*sin(a);
}

// output must be proportional to sin(input_angle)
cpFloat NEATSpatialSinRotation::evaluateSystem(ExtendedSystemInfo *sys)
{
    cpFloat fitness = 0.0;

    size_t nSteps = sys->inputRotationAngle.size();

    std::vector<cpFloat>sinOfInput = std::vector<cpFloat>(nSteps);
    std::vector<cpFloat>normalizedOutput = std::vector<cpFloat>(nSteps);
    std::transform(sys->inputRotationAngle.begin(), sys->inputRotationAngle.end(), sinOfInput.begin(), angsin);
    
    std::transform(sys->outputRotationAngle.begin(), sys->outputRotationAngle.end(), normalizedOutput.begin(), normalize_angle);
    
    // now we assume sin(input) = c*output, and we try to minimize the error
    std::vector<cpFloat>sqDiff;
    
    
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    
    
    cpFloat inputMean = std::accumulate(sinOfInput.begin(), sinOfInput.end(), 0.0)/nSteps;
    
    cpFloat outputMean = std::accumulate(normalizedOutput.begin(), normalizedOutput.end(), 0.0)/nSteps;
    
    //

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

// output must be proportional to sin(input_angle)
cpFloat oldEval(ExtendedSystemInfo *sys)
{
    
    cpFloat fitness = 0.0;
    
    size_t nSteps = sys->inputRotationAngle.size();
    
    cpFloat correlation = 0.0;
 
    std::vector<cpFloat>sinOfInput = std::vector<cpFloat>(nSteps);
    std::vector<cpFloat>normalizedOutput = std::vector<cpFloat>(nSteps);
    std::transform(sys->inputRotationAngle.begin(), sys->inputRotationAngle.end(), sinOfInput.begin(), sin);
    
    std::transform(sys->outputRotationAngle.begin(), sys->outputRotationAngle.end(), normalizedOutput.begin(), normalize_angle);
    
    cpFloat inputMean = std::accumulate(sinOfInput.begin(), sinOfInput.end(), 0.0)/nSteps;
    
    cpFloat outputMean = std::accumulate(normalizedOutput.begin(), normalizedOutput.end(), 0.0)/nSteps;
    
    
    
    
    // correlation = sum[(x - mean(x))*(y - mean(y))] / sqrt(sum[(x - mean(x))^2] * sum[(y- mean(y))^2])
    
    cpFloat numerator = 0.0;
    cpFloat sqrXDiffSum = 0.0;
    cpFloat sqrYDiffSum = 0.0;
    
    
    for (int i=0; i<nSteps; i++) {
        float xDiff = (sinOfInput[i]-inputMean);
        float yDiff = (normalizedOutput[i]-outputMean);
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
    
    fitness = fabs(correlation) ;
    
//    if (outputVariance > 0) {
//        cpFloat distance = fabs (2.0*M_PI -meandInputdOutput);
//        fitness /= (distance+0.1)*(distance+0.1);
//    }
    
    if (fitness == 0 || inputVariance < 3e-1)
        fitness = 1e-8;
    
    
    assert(fitness == fitness);
    assert(fabs(fitness) != INFINITY);
    
    return fitness;
}

bool NEATSpatialSinRotation::goodEnoughFitness(cpFloat bestFitness)
{
    return false;// bestFitness > 0.999;
}

char* NEATSpatialSinRotation::inputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *inputBody = bestIndividual->system->partAtPosition(bestIndividual->system->inputMachinePosition)->body;
    
    snprintf(buffer, 100, "Input angle : %.3f", normalize_angle(cpBodyGetAngle(inputBody)));
    
    return buffer;
}

char* NEATSpatialSinRotation::outputDescription()
{
    if (!bestIndividual)
        return "";
    
    static char buffer[100];
    cpBody *outputBody = bestIndividual->system->partAtPosition(bestIndividual->system->outputMachinePosition)->body;
    
    snprintf(buffer, 100, "Output angle : %.3f", normalize_angle(cpBodyGetAngle(outputBody)));
    
    return buffer;
}

